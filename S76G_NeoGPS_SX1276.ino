#include <lmic.h>     //http://librarymanager/All#MCCI+LoRaWAN+LMIC+library
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <NMEAGPS.h>  //http://librarymanager/All#NeoGPS

#define SX1276_RegVersion                 0x42

// RAK7200 S76G GPIOs
#define RAK7200_S76G_BLUE_LED             PA8  // Blue LED (D2) active low
#define RAK7200_S76G_RED_LED              PA11 // Red LED (D3) active low
#define RAK7200_S76G_GREEN_LED            PA12 // Green LED (D4) active low
#define RAK7200_S76G_ADC_VBAT             PB0  // ADC connected to the battery (VBATT 1M PB0 1.5M GND) 1.5M / (1M + 1.5M) = 0.6
#define RAK7200_S76G_LIS3DH_INT1          PA0  // LIS3DH (U5) (I2C address 0x19) Interrupt INT1
#define RAK7200_S76G_LIS3DH_INT2          PB5  // LIS3DH (U5) (I2C address 0x19) Interrupt INT2
#define RAK7200_S76G_LPS_INT              PA5  // LPS22HB (U7) (I2C address 0x5C) Interrupt (mutually exclusive with SPI1_NSS)
#define RAK7200_S76G_MPU_INT              PA5  // MPU9250 (U8) (I2C address 0x68) Interrupt (mutually exclusive with SPI1_CLK)
#define RAK7200_S76G_TP4054_CHG1          PB1  // ADC TP4054 (U3)
#define RAK7200_S76G_TP4054_CHG2          PB8  // ADC TP4054 (U3)

// AcSiP S7xx UART1 (Console)
#define S7xx_CONSOLE_TX                   PA9  // UART1 (CH340E U1)
#define S7xx_CONSOLE_RX                   PA10 // UART1 (CH340E U1)

// AcSiP S7xx Internal SPI2 STM32L073RZ(U|Y)x <--> SX127x
#define S7xx_SX127x_MOSI                  PB15 // SPI2
#define S7xx_SX127x_MISO                  PB14 // SPI2
#define S7xx_SX127x_SCK                   PB13 // SPI2
#define S7xx_SX127x_NSS                   PB12 // SPI2
#define S7xx_SX127x_NRESET                PB10
#define S7xx_SX127x_DIO0                  PB11
#define S7xx_SX127x_DIO1                  PC13
#define S7xx_SX127x_DIO2                  PB9
#define S7xx_SX127x_DIO3                  PB4  // unused
#define S7xx_SX127x_DIO4                  PB3  // unused
#define S7xx_SX127x_DIO5                  PA15 // unused
#define S7xx_SX127x_ANTENNA_SWITCH_RXTX   PA1  // Radio Antenna Switch 1:RX, 0:TX

// AcSiP S7xG SONY CXD5603GF GNSS
#define RAK7200_S76G_CXD5603_POWER_ENABLE PC4  // Enable 1V8 Power to GNSS (U2 TPS62740)
#define T_Motion_S76G_CXD5603_1PPS        PB5  // TTGO T-Motion 1PPS
#define S7xG_CXD5603_RESET                PB2  // Reset does not appear to work
#define S7xG_CXD5603_LEVEL_SHIFTER        PC6
#define S7xG_CXD5603_UART_TX              PC10 // UART4
#define S7xG_CXD5603_UART_RX              PC11 // UART4
#define S7xG_CXD5603_BAUD_RATE            115200

// AcSiP S7xx I2C1
#define S7xx_I2C_SCL                      PB6  // I2C1
#define S7xx_I2C_SDA                      PB7  // I2C1

// Defined Serial Port for NeoGPS Library
#define gpsPort GNSS
#define GPS_PORT_NAME "GNSS"
#define DEBUG_PORT Serial

HardwareSerial GNSS(S7xG_CXD5603_UART_RX, S7xG_CXD5603_UART_TX);

// Configure the three OTAA keys here or in an external file and #include that file
/*
 * The Device EUI (DEVEUI) must be in least-significant-byte order.
 * When copying the Device EUI from the Helium Console be sure lsb: is the byte order selected.
 */
//static const u1_t PROGMEM DEVEUI[8]= {0x00, 0x00, 0x00, 0xFE, 0xFF, 0x09, 0x1F, 0xAC};
//void os_getDevEui(u1_t *buf) {memcpy_P(buf, DEVEUI, 8);}

/*
 * The App EUI (APPEUI) must be in least-significant-byte order.
 * When copying the App EUI from the Helium Console be sure lsb: is the byte order selected.
 */
//static const u1_t PROGMEM APPEUI[8]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//void os_getArtEui(u1_t *buf) {memcpy_P(buf, APPEUI, 8);}

/*
 * The App Key (APPKEY) must be in most-significant-byte order.
 * When copying the App Key from the Helium Console be sure msb: is the byte order selected.
 */
//static const u1_t PROGMEM APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//void os_getDevKey(u1_t *buf) {memcpy_P(buf, APPKEY, 16);}

// Set OTAA keys outside of the project in my case in the directory called OTAA_Keys where the Arduino sketchbooks are stored
//#include "../../../OTAA_Keys/RAK7200-OTAA-keys"
#include "../../../OTAA_Keys/RAK7200-36F0"


static uint8_t LoRaPacketData[24] = "";
static uint8_t LoRaPacketDataSize = 0;
static osjob_t sendjob;

const unsigned TX_INTERVAL = 15; // Transmit every 15 seconds

const lmic_pinmap lmic_pins = {
        .nss = S7xx_SX127x_NSS,
        .rxtx = S7xx_SX127x_ANTENNA_SWITCH_RXTX,
        .rst = S7xx_SX127x_NRESET,
        .dio = {S7xx_SX127x_DIO0, S7xx_SX127x_DIO1, S7xx_SX127x_DIO2},
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 1000000
};

static NMEAGPS gps;
static gps_fix fix;
static byte blueLEDstate = HIGH; // Set LED state off
static byte redLEDstate = HIGH;
static byte greenLEDstate = HIGH;

static bool GNSS_probe() {
    unsigned long startTime = millis();
    char c1, c2;
    c1 = c2 = 0;

    GNSS.flush();
    while (millis() - startTime < 3000) {
        if (GNSS.available() > 0) {
            c1 = GNSS.read();
            if ((c1 == '$') && (c2 == 0)) {
                c2 = c1;
                continue;
            }
            if ((c2 == '$') && (c1 == 'G')) {
                // got $G leave the function with GNSS port opened
                return true;
            }
            else {
                c2 = 0;
            }
        }
        delay(1);
    }
    return false;
}

static void SerialPassThrough(void) {
    if (Serial.available()) {
        GNSS.write(Serial.read());
    }
    if (GNSS.available()) {
        Serial.write(GNSS.read());
    }
}

static void scanI2Cbus(void) {
    byte err, addr;
    int nDevices = 0;

    Serial.println("Scanning I2C bus");
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("I2C device found at address 0x");
            if (addr < 16) {
                Serial.print("0");
            }
            Serial.println(addr, HEX);
            nDevices++;
        }
        else {
            if (err == 4) {
                Serial.print("Unknown error at address 0x");
                if (addr < 16) {
                    Serial.print("0");
                }
                Serial.println(addr, HEX);
            }
        }
    }
    if (nDevices == 0) {
        Serial.println("No I2C devices found\n");
    }
    Serial.println("Scanning complete\n");
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16) {
        Serial.print('0');
    }
    Serial.print(v, HEX);
}

void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
                u4_t netid = 0;
                devaddr_t devaddr = 0;
                u1_t nwkKey[16];
                u1_t artKey[16];
                LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                Serial.print("netid: ");
                Serial.println(netid, DEC);
                Serial.print("devaddr: ");
                Serial.println(__builtin_bswap32(devaddr), HEX);
                Serial.print("AppSKey: ");
                for (size_t i = 0; i < sizeof(artKey); ++i) {
                    if (i != 0) {
                        Serial.print("-");
                    }
                    printHex2(artKey[i]);
                }
                Serial.println("");
                Serial.print("NwkSKey: ");
                for (size_t i = 0; i < sizeof(nwkKey); ++i) {
                    if (i != 0) {
                        Serial.print("-");
                    }
                    printHex2(nwkKey[i]);
                }
                Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
            /*
            || This event is defined but not used in the code. No
            || point in wasting codespace on it.
            ||
            || case EV_RFU1:
            ||     Serial.println(F("EV_RFU1"));
            ||     break;
            */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F("Received ack"));
            }
            if (LMIC.dataLen) {
                Serial.println(F("Received "));
                Serial.println(LMIC.dataLen);
                Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
            /*
            || This event is defined but not used in the code. No
            || point in wasting codespace on it.
            ||
            || case EV_SCAN_FOUND:
            ||    Serial.println(F("EV_SCAN_FOUND"));
            ||    break;
            */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t *j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else {
        LMIC_setTxData2(1, LoRaPacketData, LoRaPacketDataSize, 0);
        LoRaPacketDataSize = 0;
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    bool has_SX1276 = false;
    bool has_GNSS = false;
    time_t serialStart = 0;

    pinMode(RAK7200_S76G_BLUE_LED, OUTPUT);
    digitalWrite(RAK7200_S76G_BLUE_LED, blueLEDstate);
    pinMode(RAK7200_S76G_RED_LED, OUTPUT);
    digitalWrite(RAK7200_S76G_RED_LED, !redLEDstate);
    pinMode(RAK7200_S76G_GREEN_LED, OUTPUT);
    digitalWrite(RAK7200_S76G_GREEN_LED, greenLEDstate);
    pinMode(RAK7200_S76G_ADC_VBAT, INPUT);

    // Configure AcSiP S7xx Serial1 to Arduino Serial
    Serial.setTx(S7xx_CONSOLE_TX);
    Serial.setRx(S7xx_CONSOLE_RX);
    Serial.begin(115200);
    serialStart = millis();
    while (!Serial) {
        if ((millis() - serialStart) < 3000) {
            delay(100);
        }
        else {
            break;
        }
    }
    delay(3000);
    Serial.println("\nS76G Power-on Self Test\n");
    Serial.flush();

    Wire.setSCL(S7xx_I2C_SCL);
    Wire.setSDA(S7xx_I2C_SDA);
    Wire.begin();
    scanI2Cbus();
    Wire.end();

    SPI.setMISO(S7xx_SX127x_MISO);
    SPI.setMOSI(S7xx_SX127x_MOSI);
    SPI.setSCLK(S7xx_SX127x_SCK);
    SPI.setSSEL(S7xx_SX127x_NSS);

    SPI.begin();

    digitalWrite(S7xx_SX127x_NSS, HIGH);
    pinMode(S7xx_SX127x_NSS, OUTPUT);

    digitalWrite(S7xx_SX127x_NRESET, HIGH);
    pinMode(S7xx_SX127x_NRESET, OUTPUT);

    // manually reset radio
    digitalWrite(S7xx_SX127x_NRESET, LOW);
    delay(5);
    digitalWrite(S7xx_SX127x_NRESET, HIGH);
    delay(5);

    digitalWrite(S7xx_SX127x_NSS, LOW);

    SPI.transfer(SX1276_RegVersion & 0x7F);
    has_SX1276 = (SPI.transfer(0x00) == 0x12 ? true : false);

    digitalWrite(S7xx_SX127x_NSS, HIGH);

    SPI.end();
    pinMode(S7xx_SX127x_NSS, INPUT);
    pinMode(S7xx_SX127x_NRESET, INPUT);

    Serial.println("Built-in components:");

    Serial.print("RADIO  - ");
    Serial.println(has_SX1276 ? "PASS" : "FAIL");

    // power on GNSS
    pinMode(RAK7200_S76G_CXD5603_POWER_ENABLE, OUTPUT);
    digitalWrite(RAK7200_S76G_CXD5603_POWER_ENABLE, HIGH);
    delay(1200); // Delay 315µs to 800µs ramp up time

    GNSS.begin(S7xG_CXD5603_BAUD_RATE);

    /* drive GNSS RST pin low */
    pinMode(S7xG_CXD5603_RESET, OUTPUT);
    digitalWrite(S7xG_CXD5603_RESET, LOW);

    /* activate 1.8V<->3.3V level shifters */
    pinMode(S7xG_CXD5603_LEVEL_SHIFTER, OUTPUT);
    digitalWrite(S7xG_CXD5603_LEVEL_SHIFTER, HIGH);

    /* keep RST low to ensure proper IC reset */
    delay(250);

    /* release */
    digitalWrite(S7xG_CXD5603_RESET, HIGH);

    /* give Sony GNSS few ms to warm up */
    delay(125);

    /* configure GNSS */
    GNSS.write("@GCD\r\n"); // Cold start
    delay(500);
    //GNSS.write("@GSW\r\n"); // Warm start
    //delay(500);
    //GNSS.write("@GSP\r\n"); // Hot start for position accuracy
    //delay(500);
    //GNSS.write("@GPPS 0x1\r\n"); // Enable PPS
    //delay(125);
    /*
     * @GNS Select the satellite systems to be used for positioning
     * bit 0 : GPS          0x01
     * bit 1 : GLONASS      0x02
     * bit 2 : SBAS         0x04
     * bit 3 : QZSS L1-CA   0x08
     *
     */
    //GNSS.write("@GNS 0x7\r\n"); // Configure GPS, GLONASS, SBAS
    GNSS.write("@GNS 0x5\r\n"); // Configure GPS, SBAS
    //GNSS.write("@GNS 0x1\r\n"); // Configure GPS
    //GNSS.write("@GNS 0x2\r\n"); // Configure GLONASS
    delay(125);
    /*
     *
     * @BSSL Select NMEA sentences to output
     * bit0 : GGA 0x01
     * bit1 : GLL 0x02
     * bit2 : GSA 0x04
     * bit3 : GSV 0x08
     * bit4 : GNS 0x10
     * bit5 : RMC 0x20
     * bit6 : VTG 0x40
     * bit7 : ZDA 0x80
     *
     */
    //GNSS.write("@BSSL 0xFF\r\n"); // All NMEA sentences
    //GNSS.write("@BSSL 0xFE\r\n"); // All NMEA sentences but GGA
    //GNSS.write("@BSSL 0xB3\r\n"); // GGA, GLL, GNS, RMC, ZDA
    //GNSS.write("@BSSL 0xA1\r\n"); // GGA, RMC, ZDA
    GNSS.write("@BSSL 0x21\r\n"); // GGA and RMC
    delay(125);

    has_GNSS = GNSS_probe();
    Serial.print("GNSS   - ");
    Serial.println(has_GNSS ? "PASS" : "FAIL");

    Serial.println();
    Serial.println("POST is completed.\n\n");

    gpsPort.begin(115200);
    while (!gpsPort);
    Serial.println("GNSS UART Initialized");

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // allow much more clock error than the X/1000 default. See:
    // https://github.com/mcci-catena/arduino-lorawan/issues/74#issuecomment-462171974
    // https://github.com/mcci-catena/arduino-lmic/commit/42da75b56#diff-16d75524a9920f5d043fe731a27cf85aL633
    // the X/1000 means an error rate of 0.1%; the above issue discusses using values up to 10%.
    // so, values from 10 (10% error, the most lax) to 1000 (0.1% error, the most strict) can be used.
    LMIC_setClockError(1 * MAX_CLOCK_ERROR / 40);

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC_selectSubBand(1);
    Serial.println("Radio Initialized");

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
    //SerialPassThrough();
    while (gps.available(gpsPort)) {
        fix = gps.read();
    }
    if (fix.valid.location) {
        fix.valid.location = false;
        greenLEDstate = !greenLEDstate;
        digitalWrite(RAK7200_S76G_GREEN_LED, greenLEDstate);
/*
        Serial.print("Location: ");
        Serial.print(fix.latitudeL());
        Serial.print(',');
        Serial.print(fix.longitudeL());
        if (fix.valid.altitude) {
            Serial.print(", Altitude: ");
            Serial.print(fix.altitude_cm());
        }
        Serial.print(", Satellites: ");
        Serial.print(fix.satellites);
        Serial.print(", HDOP: ");
        Serial.print(fix.hdop);
        Serial.println();
*/
        // Prepare upstream data transmission at the next possible time.
        uint32_t i = 0;
        int32_t data = 0;
        data = (int32_t)(fix.latitudeL());
        Serial.print("Location: ");
        Serial.print(data);
        LoRaPacketData[i++] = data >> 24;
        LoRaPacketData[i++] = data >> 16;
        LoRaPacketData[i++] = data >> 8;
        LoRaPacketData[i++] = data;
        data = (int32_t)(fix.longitudeL());
        Serial.print(", ");
        Serial.print(data);
        LoRaPacketData[i++] = data >> 24;
        LoRaPacketData[i++] = data >> 16;
        LoRaPacketData[i++] = data >> 8;
        LoRaPacketData[i++] = data;
        data = (int32_t)(fix.altitude_cm());
        Serial.print(", Altitude: ");
        Serial.print(data);
        LoRaPacketData[i++] = data >> 24;
        LoRaPacketData[i++] = data >> 16;
        LoRaPacketData[i++] = data >> 8;
        LoRaPacketData[i++] = data;
        data = (uint32_t)(0);
        Serial.print(", Accuracy: ");
        Serial.print(data);
        LoRaPacketData[i++] = data >> 24;
        LoRaPacketData[i++] = data >> 16;
        LoRaPacketData[i++] = data >> 8;
        LoRaPacketData[i++] = data;
        data = (uint8_t)(fix.satellites);
        Serial.print(", Satellites: ");
        Serial.print(data);
        LoRaPacketData[i++] = data;
        data = (uint16_t)(fix.hdop);
        Serial.print(", HDOP: ");
        Serial.print(data);
        LoRaPacketData[i++] = data >> 8;
        LoRaPacketData[i++] = data;
        data = (int32_t)((fix.speed_mph() * 100));
        //data = (int32_t)(0);
        Serial.print(", Speed: ");
        Serial.print(data);
        LoRaPacketData[i++] = data >> 24;
        LoRaPacketData[i++] = data >> 16;
        LoRaPacketData[i++] = data >> 8;
        LoRaPacketData[i++] = data;
        LoRaPacketDataSize = i;
        Serial.print(", V: ");
        Serial.print(float(analogRead(RAK7200_S76G_ADC_VBAT)) / 4096 * 3.30 / 0.6 * 10.0);
        Serial.println();
    }
}
