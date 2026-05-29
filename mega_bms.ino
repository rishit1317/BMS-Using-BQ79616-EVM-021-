#include <Arduino.h>

/*
=========================================================
BQ79616-Q1 DAISY CHAIN UART ESTABLISHMENT
HOST   : Arduino Mega 2560
STACK  : 1 Base (address 0) + 1 Stack (address 1)

CONNECTIONS (MEGA to BASE EVM J17):
MEGA TX1 pin 18 ----> J17 pin 8  (BQ RX)
MEGA RX1 pin 19 ----> J17 pin 7  (BQ TX)
MEGA GND        ----> J17 pin 5  (GND)
MEGA 3.3V       ----> J17 pin 6  (3.3V)

BASE EVM J17 JUMPERS:
J2  populated
J18 populated
J21 populated
J1  removed

STACK EVM JUMPERS:
J1  populated
J2  removed
J18 removed
J21 removed

DAISY CHAIN:
BASE J11 pin 3 ----> STACK J10 pin 2
BASE J11 pin 4 ----> STACK J10 pin 1

=========================================================

FRAME FORMAT (TI SLVAE86B):

BROADCAST WRITE 1 BYTE:
[D0][REG_HI][REG_LO][DATA][CRC_LO][CRC_HI]

SINGLE DEVICE WRITE 1 BYTE:
[90][DEV_ADDR][REG_HI][REG_LO][DATA][CRC_LO][CRC_HI]

BROADCAST READ:
[C0][REG_HI][REG_LO][N-1][CRC_LO][CRC_HI]

SINGLE DEVICE READ:
[80][DEV_ADDR][REG_HI][REG_LO][N-1][CRC_LO][CRC_HI]

=========================================================
*/

#define BQ_TX_PIN  18
#define BQ_BAUD    1000000UL

/* =========================================================
   CRC-16-IBM
   Poly   : 0x8005 reflected = 0xA001
   Init   : 0x0000
   RefIn  : true
   RefOut : true

   VERIFIED against TI document frames:
   D0 03 4C 00 -> FC 24  PASS
   D0 03 09 01 -> 0F 74  PASS
   D0 03 06 00 -> CB 44  PASS
   D0 03 06 01 -> 0A 84  PASS
   D0 03 08 02 -> 4E E5  PASS
   90 00 03 08 00 -> 13 DD  PASS
   C0 03 4C 00 -> F8 E4  PASS
   80 00 02 15 0B -> CB 49  PASS (TI Table 1-2)
========================================================= */

uint16_t calcCRC(uint8_t *data, uint8_t len)
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

/* =========================================================
   SEND RAW FRAME
   Sends exact bytes including pre-calculated CRC
========================================================= */

void sendRaw(uint8_t *data, uint8_t len)
{
    /* flush stale RX */
    while (Serial1.available())
        Serial1.read();

    delayMicroseconds(500);

    Serial1.write(data, len);
    Serial1.flush();

    delayMicroseconds(500);
}

/* =========================================================
   RECEIVE RESPONSE
   Waits up to timeoutMs
   Resets timeout on each new byte
========================================================= */

uint8_t receiveResponse(uint8_t  *buf,
                        uint8_t   maxLen,
                        uint16_t  timeoutMs)
{
    uint32_t start = millis();
    uint8_t  count = 0;

    while ((millis() - start) < timeoutMs)
    {
        if (Serial1.available())
        {
            buf[count++] = Serial1.read();

            if (count >= maxLen)
                break;

            start = millis();
        }
    }

    return count;
}

/* =========================================================
   PRINT RESPONSE
========================================================= */

void printResponse(const char *label,
                   uint8_t    *buf,
                   uint8_t     len)
{
    Serial.print(label);

    if (len == 0)
    {
        Serial.println(" -> NO RESPONSE");
        return;
    }

    Serial.print(" -> ");
    Serial.print(len);
    Serial.print(" bytes: ");

    for (uint8_t i = 0; i < len; i++)
    {
        if (buf[i] < 0x10)
            Serial.print("0");
        Serial.print(buf[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
}

/* =========================================================
   VERIFY CRC
   Run this first to confirm CRC function is correct
   All results must match TI document exactly
========================================================= */

void verifyCRC()
{
    Serial.println("--- CRC VERIFICATION ---");

    struct
    {
        const char *label;
        uint8_t     data[8];
        uint8_t     len;
        uint16_t    expected;
    }
    tests[] =
    {
        /* TI document SLVAE86B section 4.2 */
        { "D0 03 4C 00",          { 0xD0,0x03,0x4C,0x00 },             4, 0x24FC },
        { "D0 03 09 01",          { 0xD0,0x03,0x09,0x01 },             4, 0x740F },
        { "D0 03 06 00",          { 0xD0,0x03,0x06,0x00 },             4, 0x44CB },
        { "D0 03 06 01",          { 0xD0,0x03,0x06,0x01 },             4, 0x840A },
        { "D0 03 08 02",          { 0xD0,0x03,0x08,0x02 },             4, 0xE54E },
        { "90 00 03 08 00",       { 0x90,0x00,0x03,0x08,0x00 },        5, 0xDD13 },
        { "C0 03 4C 00",          { 0xC0,0x03,0x4C,0x00 },             4, 0xE4F8 },
        /* TI document Table 1-2 */
        { "80 00 02 15 0B",       { 0x80,0x00,0x02,0x15,0x0B },        5, 0x49CB },
        /* our frames */
        { "90 01 03 08 03",       { 0x90,0x01,0x03,0x08,0x03 },        5, 0x9CD2 },
        { "80 00 05 00 00",       { 0x80,0x00,0x05,0x00,0x00 },        5, 0x192A },
    };

    bool allPass = true;

    for (uint8_t i = 0; i < sizeof(tests)/sizeof(tests[0]); i++)
    {
        uint16_t got = calcCRC(tests[i].data, tests[i].len);

        bool pass = (got == tests[i].expected);

        if (!pass)
            allPass = false;

        Serial.print("  ");
        Serial.print(tests[i].label);
        Serial.print(" -> got 0x");
        if (got < 0x1000) Serial.print("0");
        if (got < 0x100)  Serial.print("0");
        if (got < 0x10)   Serial.print("0");
        Serial.print(got, HEX);
        Serial.print(" expect 0x");
        if (tests[i].expected < 0x1000) Serial.print("0");
        if (tests[i].expected < 0x100)  Serial.print("0");
        if (tests[i].expected < 0x10)   Serial.print("0");
        Serial.print(tests[i].expected, HEX);
        Serial.println(pass ? "  PASS" : "  FAIL <<<");
    }

    Serial.println(allPass ? "ALL CRC PASS" : "CRC FAILURE - DO NOT PROCEED");
    Serial.println();
}

/* =========================================================
   WAKE SEQUENCE
   TI SLVAE86B Section 3
   Single LOW pulse on TX1 pin for 3ms
   Wait 100ms for both devices to wake
========================================================= */

void wakeBQ()
{
    Serial1.end();

    pinMode(BQ_TX_PIN, OUTPUT);

    digitalWrite(BQ_TX_PIN, HIGH);
    delay(10);

    /* WAKE PULSE — LOW for 3ms */
    digitalWrite(BQ_TX_PIN, LOW);
    delay(3);
    digitalWrite(BQ_TX_PIN, HIGH);

    /* wait for both devices */
    delay(100);

    Serial1.begin(BQ_BAUD);

    delay(10);
    while (Serial1.available())
        Serial1.read();
}

/* =========================================================
   AUTO ADDRESSING — 2 DEVICES

   ALL FRAMES FROM TI SLVAE86B SECTION 4.2
   CRC SENT AS [CRC_LO][CRC_HI] (LSB FIRST)

   CRC value 0xXXYY is sent as byte XX then byte YY
   Example: CRC = 0x24FC -> send FC then 24

   STEP 1: D0 03 4C 00 | FC 24
   STEP 2: D0 03 09 01 | 0F 74
   STEP 3a: D0 03 06 00 | CB 44
   STEP 3b: D0 03 06 01 | 0A 84
   STEP 4: D0 03 08 02 | 4E E5
   STEP 5a: 90 00 03 08 00 | 13 DD
   STEP 5b: 90 01 03 08 03 | D2 9C
   STEP 6: C0 03 4C 00 | F8 E4
========================================================= */

void autoAddress()
{
    uint8_t rxBuf[64];
    uint8_t n;

    Serial.println("  STEP 1: DLL sync write");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x4C, 0x00, 0xFC, 0x24 };
        sendRaw(f, sizeof(f));
        delay(10);
    }

    Serial.println("  STEP 2: Enable auto-addressing");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x09, 0x01, 0x0F, 0x74 };
        sendRaw(f, sizeof(f));
        delay(10);
    }

    Serial.println("  STEP 3a: Assign address 0");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x06, 0x00, 0xCB, 0x44 };
        sendRaw(f, sizeof(f));
        delay(10);
    }

    Serial.println("  STEP 3b: Assign address 1");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x06, 0x01, 0x0A, 0x84 };
        sendRaw(f, sizeof(f));
        delay(10);
    }

    Serial.println("  STEP 4: Set all as stack");
    {
        uint8_t f[] = { 0xD0, 0x03, 0x08, 0x02, 0x4E, 0xE5 };
        sendRaw(f, sizeof(f));
        delay(10);
    }

    Serial.println("  STEP 5a: Set device 0 as base");
    {
        uint8_t f[] = { 0x90, 0x00, 0x03, 0x08, 0x00, 0x13, 0xDD };
        sendRaw(f, sizeof(f));
        delay(10);
    }

    Serial.println("  STEP 5b: Set device 1 as top of stack");
    {
        /*
         * CRC over {90 01 03 08 03}
         * calcCRC = 0x9CD2
         * sent LSB first: D2 then 9C
         */
        uint8_t f[] = { 0x90, 0x01, 0x03, 0x08, 0x03, 0xD2, 0x9C };
        sendRaw(f, sizeof(f));
        delay(10);
    }

    Serial.println("  STEP 6: DLL sync read");
    {
        uint8_t f[] = { 0xC0, 0x03, 0x4C, 0x00, 0xF8, 0xE4 };
        sendRaw(f, sizeof(f));
        n = receiveResponse(rxBuf, sizeof(rxBuf), 500);
        printResponse("  DLL sync response", rxBuf, n);
    }

    delay(10);
}

/* =========================================================
   READ PART ID — DEVICE 0
   
   FROM TI TABLE 1-2:
   Frame: 80 00 02 15 0B CB 49
   reads 12 bytes from register 0x0215

   But for PARTID specifically we read register 0x0500:
   Frame: 80 00 05 00 00 2A 19
   80   = single device read
   00   = device address 0
   05 00 = register 0x0500 (PARTID)
   00   = read 1 byte (N-1 = 0)
   2A 19 = CRC over {80 00 05 00 00} = 0x192A
========================================================= */

void readPartID(uint8_t devAddr)
{
    uint8_t rxBuf[16];

    Serial.print("  Part ID read device ");
    Serial.println(devAddr);

    uint8_t f[] =
    {
        0x80,       /* single device read                    */
        devAddr,    /* device address                        */
        0x05,       /* register 0x0500 high byte             */
        0x00,       /* register 0x0500 low byte              */
        0x00,       /* N-1 = 0 means read 1 byte             */
        0x2A,       /* CRC low byte  (valid for addr 0x00)   */
        0x19        /* CRC high byte (valid for addr 0x00)   */
    };

    sendRaw(f, sizeof(f));

    uint8_t n = receiveResponse(rxBuf, sizeof(rxBuf), 500);

    printResponse("  Part ID response", rxBuf, n);
}

/* =========================================================
   SINGLE DEVICE READ — GENERIC
   
   FROM TI TABLE 1-2:
   [80][DEV_ADDR][REG_HI][REG_LO][N-1][CRC_LO][CRC_HI]

   80       = single device read init byte
   DEV_ADDR = device address (0x00 for base)
   REG_HI   = register address high byte
   REG_LO   = register address low byte
   N-1      = number of bytes to read minus 1
   CRC      = CRC-16-IBM over all preceding bytes
              sent LSB first then MSB

   TI Table 1-2 example:
   Read 12 bytes from 0x0215 on device 0:
   80 00 02 15 0B CB 49

   CRC verified: calcCRC({80,00,02,15,0B}) = 0x49CB
   Sent as CB then 49 = CB 49  MATCHES TI TABLE
========================================================= */

void singleDeviceRead(uint8_t  devAddr,
                      uint16_t regAddr,
                      uint8_t  numBytes)
{
    uint8_t rxBuf[140];
    uint8_t payload[5];

    /* build payload without CRC first */
    payload[0] = 0x80;                        /* single device read    */
    payload[1] = devAddr;                     /* device address        */
    payload[2] = (uint8_t)(regAddr >> 8);     /* register high byte    */
    payload[3] = (uint8_t)(regAddr & 0xFF);   /* register low byte     */
    payload[4] = numBytes - 1;                /* N-1                   */

    /* calculate CRC over payload */
    uint16_t crc    = calcCRC(payload, 5);
    uint8_t  crcLo  = crc & 0xFF;
    uint8_t  crcHi  = (crc >> 8) & 0xFF;

    /* build full frame */
    uint8_t frame[7];
    frame[0] = payload[0];
    frame[1] = payload[1];
    frame[2] = payload[2];
    frame[3] = payload[3];
    frame[4] = payload[4];
    frame[5] = crcLo;
    frame[6] = crcHi;

    /* print what we are sending */
    Serial.print("  TX: ");
    for (uint8_t i = 0; i < 7; i++)
    {
        if (frame[i] < 0x10) Serial.print("0");
        Serial.print(frame[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    sendRaw(frame, 7);

    uint8_t n = receiveResponse(rxBuf,
                                sizeof(rxBuf),
                                500);

    printResponse("  RX", rxBuf, n);
}

/* =========================================================
   SETUP
========================================================= */

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    delay(3000);

    Serial.println();
    Serial.println("================================");
    Serial.println("BQ79616 DAISY CHAIN TEST");
    Serial.println("1 BASE + 1 STACK");
    Serial.println("================================");
    Serial.println();

    /* ---- STEP 0: VERIFY CRC FUNCTION ---- */
    Serial.println("STEP 0: CRC VERIFICATION");
    verifyCRC();

    /* ---- STEP 1: WAKE ---- */
    Serial.println("STEP 1: WAKE");
    wakeBQ();
    Serial.println("  Wake complete");
    Serial.println();

    /* ---- STEP 2: AUTO ADDRESS ---- */
    Serial.println("STEP 2: AUTO ADDRESS");
    autoAddress();
    Serial.println("  Auto address complete");
    Serial.println();

    /* ---- STEP 3: READ PART ID ---- */
    Serial.println("STEP 3: READ PART ID (register 0x0500, 1 byte)");
    readPartID(0x00);
    Serial.println();

    /* ---- STEP 4: SINGLE DEVICE READ TI TABLE 1-2 EXAMPLE ---- */
    /*
       Read 12 bytes from register 0x0215 on device 0
       This is the EXACT example from TI Table 1-2
       Frame should be: 80 00 02 15 0B CB 49
    */
    Serial.println("STEP 4: SINGLE DEVICE READ (TI Table 1-2 example)");
    Serial.println("  Reading 12 bytes from reg 0x0215 device 0");
    singleDeviceRead(0x00, 0x0215, 12);
    Serial.println();

    /* ---- STEP 5: SINGLE DEVICE READ PARTID USING GENERIC FUNCTION ---- */
    Serial.println("STEP 5: SINGLE DEVICE READ PARTID via generic function");
    Serial.println("  Reading 1 byte from reg 0x0500 device 0");
    singleDeviceRead(0x00, 0x0500, 1);
    Serial.println();

    Serial.println("================================");
    Serial.println("DONE");
    Serial.println("================================");
}

/* =========================================================
   LOOP
========================================================= */

void loop()
{
}