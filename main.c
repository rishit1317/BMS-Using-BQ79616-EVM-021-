/* =============================================================================
 * main.c — BQ79616 Battery Monitor Driver
 * Target : STM32F103C8T6 (Blue Pill) @ 72 MHz
 *
 * CONNECTIONS:
 * PA9  (TX) ----> BQ79616 J17 pin 8 (RX of BQ)
 * PA10 (RX) ----> BQ79616 J17 pin 7 (TX of BQ)
 * GND       ----> J17 pin 5
 * 3.3V      ----> J17 pin 6
 * PA2  (TX) ----> USB-TTL RX (debug serial monitor at 115200)
 *
 * BASE EVM JUMPERS:
 * J2  populated
 * J18 populated
 * J21 populated
 * J1  removed
 *
 * STACK EVM JUMPERS:
 * J1  populated
 * J2  removed
 * J18 removed
 * J21 removed
 *
 * DAISY CHAIN:
 * BASE  J11 pin 3 (COMH_P) ----> STACK J10 pin 2 (COML_P)
 * BASE  J11 pin 4 (COMH_N) ----> STACK J10 pin 1 (COML_N)
 *
 * CLOCK:
 * HSE 8MHz x9 PLL = 72MHz
 * APB1 = 36MHz
 * APB2 = 72MHz (USART1 clock)
 * =============================================================================
 */

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

/* =============================================================================
 * SECTION 1 — DEFINES
 * =============================================================================
 */

#define BQ_TX_PORT      GPIOA
#define BQ_TX_PIN       GPIO_PIN_9

#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13

#define BQ_UART_TIMEOUT 200
#define BQ_RX_MAX       140

/* Register addresses from TI datasheet */
#define REG_OTP_ECC_TEST    0x034C
#define REG_DIR0_ADDR       0x0306
#define REG_COMM_CTRL       0x0308
#define REG_CONTROL1        0x0309
#define REG_PARTID          0x0500

/* =============================================================================
 * SECTION 2 — GLOBALS
 * =============================================================================
 */

UART_HandleTypeDef huart1;   /* BQ79616 — 1MHz                */
UART_HandleTypeDef huart2;   /* Debug   — 115200 to PC        */

static char dbg_buf[128];    /* scratch buffer for debug text */
volatile uint32_t g_wake_done        = 0;
volatile uint32_t g_commclear_done   = 0;
volatile uint32_t g_autoaddr_done    = 0;
volatile uint32_t g_partid_ok        = 0;

/* last received bytes and count */
volatile uint8_t  g_last_rx_buf[32]  = {0};
volatile uint8_t  g_last_rx_count    = 0;

/* step tracker — shows exactly which step code is on */
volatile uint32_t g_step             = 0;

/* UART error tracking */
volatile uint32_t g_uart_error_code  = 0;

/* =============================================================================
 * SECTION 3 — FUNCTION PROTOTYPES
 * =============================================================================
 */

void     SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
void     BQ_InitDWT(void);
void     delay_us(uint32_t us);
uint16_t BQ_CRC16(uint8_t *data, uint8_t len);
void     BQ_Wake(void);
void     BQ_CommClear(void);
void     BQ_SendRaw(uint8_t *frame, uint8_t len);
uint8_t  BQ_Receive(uint8_t *buf, uint8_t maxLen, uint32_t timeoutMs);
void     BQ_AutoAddress_2Dev(void);
bool     BQ_ReadPartID(void);
void     dbg(const char *msg);

/* =============================================================================
 * SECTION 4 — DEBUG HELPER
 * Sends string to USART2 PA2 at 115200
 * Connect PA2 to USB-TTL RX pin for serial monitor
 * =============================================================================
 */
void dbg(const char *msg)
{
    HAL_UART_Transmit(&huart2,
                      (uint8_t *)msg,
                      (uint16_t)strlen(msg),
                      100);
}

/* =============================================================================
 * SECTION 5 — DWT MICROSECOND DELAY
 * Uses ARM DWT cycle counter for accurate us delays
 * =============================================================================
 */
void BQ_InitDWT(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0U;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

/* =============================================================================
 * SECTION 6 — CRC-16-IBM
 *
 * Parameters:
 *   Polynomial : 0x8005
 *   Init       : 0x0000   <-- CRITICAL: must be 0x0000 not 0xFFFF
 *   RefIn      : true
 *   RefOut     : true
 *   XorOut     : 0x0000
 *   Reflected poly = 0xA001
 *
 * VERIFIED against all TI SLVAE86B known frames:
 *
 *   D0 03 4C 00  ->  FC 24  PASS
 *   D0 03 09 01  ->  0F 74  PASS
 *   D0 03 06 00  ->  CB 44  PASS
 *   D0 03 06 01  ->  0A 84  PASS
 *   D0 03 08 02  ->  4E E5  PASS
 *   90 00 03 08 00  ->  13 DD  PASS
 *   C0 03 4C 00  ->  F8 E4  PASS
 *
 * New frames calculated for 2-device stack:
 *   90 01 03 08 03  ->  D2 9C
 *   80 00 05 00 00  ->  2A 19
 * =============================================================================
 */
uint16_t BQ_CRC16(uint8_t *data, uint8_t len)
{
    uint16_t crc = 0x0000;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i];

        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x0001u)
                crc = (crc >> 1) ^ 0xA001u;
            else
                crc >>= 1;
        }
    }

    return crc;
}

/* =============================================================================
 * SECTION 7 — SEND RAW FRAME
 * Sends exact bytes including pre-calculated CRC
 * Flushes RX before sending to avoid reading own TX bytes
 * =============================================================================
 */
void BQ_SendRaw(uint8_t *frame, uint8_t len)
{
    /* flush stale RX bytes */
    uint8_t dummy;
    while (HAL_UART_Receive(&huart1, &dummy, 1, 1) == HAL_OK);

    /* small gap before frame */
    delay_us(500);

    HAL_UART_Transmit(&huart1, frame, len, BQ_UART_TIMEOUT);

    /* small gap after frame */
    delay_us(500);
}

/* =============================================================================
 * SECTION 8 — RECEIVE RESPONSE
 * Polls for bytes one at a time
 * Resets overall timeout on each byte received
 * Returns total bytes received
 * =============================================================================
 */
uint8_t BQ_Receive(uint8_t *buf, uint8_t maxLen, uint32_t timeoutMs)
{
    uint8_t  count = 0;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeoutMs)
    {
        if (HAL_UART_Receive(&huart1, &buf[count], 1, 5) == HAL_OK)
        {
            count++;

            if (count >= maxLen)
                break;

            /* reset timeout on each new byte */
            start = HAL_GetTick();
        }
    }

    return count;
}

/* =============================================================================
 * SECTION 9 — WAKE SEQUENCE
 *
 * TI SLVAE86B Section 3:
 * Wake ping is active LOW on BQ RX line for minimum 2.5ms.
 * PA9 (UART TX) connects to J17 pin 8 (BQ RX).
 * We temporarily make PA9 a GPIO to pull LOW for 3ms.
 * Then restore as AF open-drain for UART.
 *
 * Post-wake delay:
 * TI formula: (10ms + 0.6ms) x num_devices
 * 2 devices = 21.2ms minimum
 * We use 100ms to be safe
 * =============================================================================
 */
void BQ_Wake(void)
{
    GPIO_InitTypeDef g = {0};

    /* disable UART */
    __HAL_UART_DISABLE(&huart1);

    /* PA9 as push-pull output */
    g.Pin   = BQ_TX_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BQ_TX_PORT, &g);

    /* idle HIGH before pulse */
    HAL_GPIO_WritePin(BQ_TX_PORT, BQ_TX_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    /* WAKE PULSE — LOW for 3ms */
    HAL_GPIO_WritePin(BQ_TX_PORT, BQ_TX_PIN, GPIO_PIN_RESET);
    HAL_Delay(3);

    /* release HIGH */
    HAL_GPIO_WritePin(BQ_TX_PORT, BQ_TX_PIN, GPIO_PIN_SET);

    /* restore PA9 as AF open-drain UART TX */
    g.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(BQ_TX_PORT, &g);

    /* re-enable UART */
    __HAL_UART_ENABLE(&huart1);

    /* wait for both devices to wake */
    HAL_Delay(100);

    /* flush garbage bytes */
    uint8_t dummy;
    while (HAL_UART_Receive(&huart1, &dummy, 1, 1) == HAL_OK);

    g_step = 1;
      g_wake_done = 1;
      dbg("WAKE COMPLETE\r\n");
}

/* =============================================================================
 * SECTION 10 — COMM CLEAR
 *
 * Holds TX LOW for 18 microseconds (18 bit-periods at 1Mbaud).
 * Resets BQ79616 UART state machine.
 * Must be called after wake and before first command frame.
 * =============================================================================
 */
void BQ_CommClear(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_UART_DISABLE(&huart1);

    /* PA9 as push-pull output */
    g.Pin   = BQ_TX_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BQ_TX_PORT, &g);

    /* LOW for 18us */
    HAL_GPIO_WritePin(BQ_TX_PORT, BQ_TX_PIN, GPIO_PIN_RESET);
    delay_us(18);

    /* release HIGH */
    HAL_GPIO_WritePin(BQ_TX_PORT, BQ_TX_PIN, GPIO_PIN_SET);
    delay_us(10);

    /* restore AF open-drain */
    g.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(BQ_TX_PORT, &g);

    /* re-enable UART */
    __HAL_UART_ENABLE(&huart1);

    /* clear hardware error flags */
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);

    /* flush RX */
    uint8_t dummy;
    while (HAL_UART_Receive(&huart1, &dummy, 1, 1) == HAL_OK);

    HAL_Delay(5);
    g_step = 1;
      g_wake_done = 1;
      dbg("WAKE COMPLETE\r\n");


}

/* =============================================================================
 * SECTION 11 — AUTO ADDRESSING — 2 DEVICES (1 BASE + 1 STACK)
 *
 * FROM TI SLVAE86B SECTION 4.2
 * ALL CRC VALUES VERIFIED
 *
 * FRAME FORMAT REMINDER:
 *
 * Broadcast write 1 byte:
 * [D0][REG_HI][REG_LO][DATA][CRC_LO][CRC_HI]
 *
 * Single device write 1 byte:
 * [90][DEV_ADDR][REG_HI][REG_LO][DATA][CRC_LO][CRC_HI]
 *
 * Broadcast read 1 byte:
 * [C0][REG_HI][REG_LO][00][CRC_LO][CRC_HI]
 *
 * STEP 1: D0 03 4C 00 FC 24
 *   Broadcast write OTP_ECC_TEST=0x00 to sync DLL
 *
 * STEP 2: D0 03 09 01 0F 74
 *   Broadcast write CONTROL1=0x01 to enable auto-addressing
 *
 * STEP 3a: D0 03 06 00 CB 44
 *   Broadcast write DIR0_ADDR=0x00 assigns address 0
 *   (first unaddressed device gets this)
 *
 * STEP 3b: D0 03 06 01 0A 84
 *   Broadcast write DIR0_ADDR=0x01 assigns address 1
 *   (next unaddressed device gets this)
 *
 * STEP 4: D0 03 08 02 4E E5
 *   Broadcast write COMM_CTRL=0x02 sets all as stack devices
 *
 * STEP 5a: 90 00 03 08 00 13 DD
 *   Single write to device 0 COMM_CTRL=0x00 sets it as base device
 *
 * STEP 5b: 90 01 03 08 03 D2 9C
 *   Single write to device 1 COMM_CTRL=0x03 sets it as top of stack
 *
 * STEP 6: C0 03 4C 00 F8 E4
 *   Broadcast read OTP_ECC_TEST to sync DLL
 *   Should receive response bytes if communication is working
 * =============================================================================
 */
void BQ_AutoAddress_2Dev(void)
{
    uint8_t rxBuf[32];
    uint8_t n;

    dbg("AUTO ADDRESS START\r\n");

    /* STEP 1 */
    g_step = 31;
    dbg("  [1/6] DLL sync write\r\n");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x4C, 0x00, 0xFC, 0x24 };
        BQ_SendRaw(f, sizeof(f));
        HAL_Delay(10);
    }

    /* STEP 2 */
    g_step = 32;
    dbg("  [2/6] Enable auto-address\r\n");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x09, 0x01, 0x0F, 0x74 };
        BQ_SendRaw(f, sizeof(f));
        HAL_Delay(10);
    }

    /* STEP 3a */
    g_step = 33;
    dbg("  [3a/6] Assign address 0\r\n");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x06, 0x00, 0xCB, 0x44 };
        BQ_SendRaw(f, sizeof(f));
        HAL_Delay(10);
    }

    /* STEP 3b */
    g_step = 34;
    dbg("  [3b/6] Assign address 1\r\n");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x06, 0x01, 0x0A, 0x84 };
        BQ_SendRaw(f, sizeof(f));
        HAL_Delay(10);
    }

    /* STEP 4 */
    g_step = 35;
    dbg("  [4/6] Set all as stack\r\n");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x08, 0x02, 0x4E, 0xE5 };
        BQ_SendRaw(f, sizeof(f));
        HAL_Delay(10);
    }

    /* STEP 5a */
    g_step = 36;
    dbg("  [5a/6] Set device 0 as base\r\n");
    {
        uint8_t f[] = { 0x90, 0x00, 0x03, 0x08, 0x00, 0x13, 0xDD };
        BQ_SendRaw(f, sizeof(f));
        HAL_Delay(10);
    }

    /* STEP 5b */
    g_step = 37;
    dbg("  [5b/6] Set device 1 as top of stack\r\n");
    {
        /*
         * CRC over: 90 01 03 08 03
         * BQ_CRC16 result = 0x9CD2
         * Sent LSB first = D2 9C
         */
        uint8_t f[] = { 0x90, 0x01, 0x03, 0x08, 0x03, 0xD2, 0x9C };
        BQ_SendRaw(f, sizeof(f));
        HAL_Delay(10);
    }

    /* STEP 6 */
    g_step = 38;
    dbg("  [6/6] DLL sync read\r\n");
    {
        uint8_t f[] = { 0xC0, 0x03, 0x4C, 0x00, 0xF8, 0xE4 };
        BQ_SendRaw(f, sizeof(f));

        n = BQ_Receive(rxBuf, sizeof(rxBuf), 500);

        snprintf(dbg_buf, sizeof(dbg_buf),
                 "  Response: %d bytes\r\n", n);
        dbg(dbg_buf);

        if (n > 0)
        {
            dbg("  Bytes: ");
            for (uint8_t i = 0; i < n; i++)
            {
                snprintf(dbg_buf, sizeof(dbg_buf), "%02X ", rxBuf[i]);
                dbg(dbg_buf);
            }
            dbg("\r\n");
        }
    }
    g_step = 39;
       g_autoaddr_done = 1;
    HAL_Delay(10);
    dbg("AUTO ADDRESS COMPLETE\r\n");
}

/* =============================================================================
 * SECTION 12 — READ PART ID — DEVICE 0
 *
 * Frame: 80 00 05 00 00 2A 19
 *   80    = single device read
 *   00    = device address 0
 *   05 00 = register PARTID (0x0500)
 *   00    = read 1 byte (N-1=0 means 1 byte)
 *   2A 19 = CRC over {80 00 05 00 00}
 *           BQ_CRC16 = 0x192A, sent as 2A 19
 *
 * Response frame from BQ79616 (7 bytes total):
 *   [INIT][DEV_ADDR][REG_HI][REG_LO][DATA][CRC_LO][CRC_HI]
 *   Byte index 4 = Part ID value
 * =============================================================================
 */
bool BQ_ReadPartID(void)
{
    uint8_t rxBuf[16];
    uint8_t n;
    g_step = 4;
    dbg("READING PART ID (device 0)\r\n");

    /*
     * CRC pre-calculated:
     * BQ_CRC16({80, 00, 05, 00, 00}) = 0x192A
     * Sent LSB first: 2A 19
     */
    uint8_t f[] = { 0x80, 0x00, 0x05, 0x00, 0x00, 0x2A, 0x19 };

    BQ_SendRaw(f, sizeof(f));

    n = BQ_Receive(rxBuf, sizeof(rxBuf), 500);
    g_last_rx_count   = n;
        g_uart_error_code = huart1.ErrorCode;

    snprintf(dbg_buf, sizeof(dbg_buf),
             "  Response: %d bytes\r\n", n);
    dbg(dbg_buf);

    if (n > 0)
    {
        dbg("  Bytes: ");
        for (uint8_t i = 0; i < n; i++)
        {
            snprintf(dbg_buf, sizeof(dbg_buf), "%02X ", rxBuf[i]);
            dbg(dbg_buf);
        }
        dbg("\r\n");
        g_partid_ok = 1;
        g_step      = 99;


        return true;
    }
    g_step = 0;

    dbg("  NO RESPONSE\r\n");
    return false;
}

/* =============================================================================
 * SECTION 13 — MAIN
 * =============================================================================
 */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    BQ_InitDWT();

    /* LED off at start — PC13 active LOW so SET = off */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    HAL_Delay(3000);

    dbg("\r\n================================\r\n");
    dbg("BQ79616 DAISY CHAIN TEST\r\n");
    dbg("STM32F103C8T6 BLUE PILL\r\n");
    dbg("1 BASE + 1 STACK\r\n");
    dbg("================================\r\n\r\n");

    /* STEP 1 — WAKE */
    dbg("STEP 1: WAKE\r\n");
    BQ_Wake();

    /* STEP 2 — COMM CLEAR */
    dbg("STEP 2: COMM CLEAR\r\n");
    BQ_CommClear();

    /* STEP 3 — AUTO ADDRESS */
    dbg("STEP 3: AUTO ADDRESS\r\n");
    BQ_AutoAddress_2Dev();

    /* STEP 4 — READ PART ID */
    dbg("STEP 4: READ PART ID\r\n");
    bool ok = BQ_ReadPartID();

    if (ok)
    {
        dbg("\r\nSUCCESS - BQ79616 RESPONDING\r\n");
        /* LED on = success */
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    }
    else
    {
        dbg("\r\nFAILED - NO RESPONSE\r\n");
        dbg("CHECK WIRING AND JUMPERS\r\n");
    }

    while (1)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(500);
    }
}

/* =============================================================================
 * SECTION 14 — CLOCK CONFIG
 * HSE 8MHz x9 PLL = 72MHz
 * =============================================================================
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    osc.HSIState       = RCC_HSI_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        Error_Handler();

    clk.ClockType      = RCC_CLOCKTYPE_HCLK  |
                         RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1  |
                         RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
        Error_Handler();
}

/* =============================================================================
 * SECTION 15 — GPIO INIT
 * PC13 = LED active LOW
 * =============================================================================
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* PC13 LED active LOW start OFF */
    g.Pin   = LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

/* =============================================================================
 * SECTION 16 — USART1 INIT — BQ79616 at 1MHz
 * PA9  TX — AF open-drain (BQ79616 half-duplex UART)
 * PA10 RX — input floating
 * =============================================================================
 */
static void MX_USART1_UART_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA9 TX open-drain */
    g.Pin   = GPIO_PIN_9;
    g.Mode  = GPIO_MODE_AF_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* PA10 RX input floating */
    g.Pin  = GPIO_PIN_10;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 1000000;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}

/* =============================================================================
 * SECTION 17 — USART2 INIT — Debug at 115200
 * PA2 TX — connect to USB-TTL RX for serial monitor
 * PA3 RX — optional
 * =============================================================================
 */
static void MX_USART2_UART_Init(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA2 TX push-pull */
    g.Pin   = GPIO_PIN_2;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /* PA3 RX input floating */
    g.Pin  = GPIO_PIN_3;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

/* =============================================================================
 * SECTION 18 — ERROR HANDLER
 * =============================================================================
 */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(100);
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
