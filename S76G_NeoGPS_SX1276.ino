#include <Wire.h>
#include <SPI.h>
#include <lmic.h>     //http://librarymanager/All#MCCI+LoRaWAN+LMIC+library
#include <hal/hal.h>
#include <GPSfix_cfg.h> // Configures NMEA Sentences for Decoding
#include <NMEAGPS.h>  //http://librarymanager/All#NeoGPS

#define SX1276_RegVersion                 0x42

// UART1 (Console)
#define S76G_CONSOLE_RX                   PA10
#define S76G_CONSOLE_TX                   PA9

// RAK7200 GPIOs
#define S76G_RAK7200_BLUE_LED             PA8  // Blue LED (D2) on RAK7200 active low
#define S76G_RAK7200_RED_LED              PA11 // Red LED (D3) on RAK7200 active low
#define S76G_RAK7200_GREEN_LED            PA12 // Green LED (D4) on RAK7200 active low
#define S76G_RAK7200_ADC_VBAT             PB0  // ADC connected to the battery (VBATT 1M PB0 1.5M GND) 1.5M / (1M + 1.5M) = 0.6
#define S76G_RAK7200_LIS3DH_INT1          PA0 // LIS3DH INT1
#define S76G_RAK7200_LIS3DH_INT2          PB5 // LIS3DH INT2

// GNSS GPIOs
#define S76G_RAK7200_GNSS_POWER_ENABLE    PC4  // RAK7200 RP104N181 1V8 Enable (LDO Regulator Pos 1.8V 0.15A 5-Pin SOT-23)
#define S76G_GNSS_RX                      PC11 // UART4_RX
#define S76G_GNSS_TX                      PC10 // UART4_TX
#define S76G_GNSS_RESET_X                 PB2
#define S76G_GNSS_LEVEL_SHIFTER           PC6
#define S76G_GNSS_1PPS                    PB5
#define GNSS_BAUD_RATE                    115200

// SX1276 (SPI2) GPIOs
#define S76G_SX1276_MOSI                  PB15
#define S76G_SX1276_MISO                  PB14
#define S76G_SX1276_SCK                   PB13
#define S76G_SX1276_SS                    PB12
#define S76G_SX1276_RESET                 PB10
#define S76G_SX1276_DIO0                  PB11
#define S76G_SX1276_DIO1                  PC13
#define S76G_SX1276_DIO2                  PB9
#define S76G_SX1276_DIO3                  PB4
#define S76G_SX1276_DIO4                  PB3
#define S76G_SX1276_DIO5                  PA15

// Antenna RF switch
#define S76G_SX1276_ANTENNA_SWITCH_RXTX   PA1 // 1:Rx, 0:Tx

// I2C1 GPIOs
#define S76G_I2C1_SDA                     PB7
#define S76G_I2C1_SCL                     PB6

#define CONSOLE_SERIAL                    Serial1
#define GNSS_SERIAL                       Serial4

// Defined Serial Port for NeoGPS Library
#define gpsPort GNSS_SERIAL
#define GPS_PORT_NAME "GNSS_SERIAL"
#define DEBUG_PORT CONSOLE_SERIAL

HardwareSerial CONSOLE_SERIAL(S76G_CONSOLE_RX, S76G_CONSOLE_TX);
HardwareSerial GNSS_SERIAL(S76G_GNSS_RX, S76G_GNSS_TX);

// Configure the three OTAA keys here or in an external file and #include that file
/*
 * The Device EUI (DEVEUI) must be in least-significant-byte order.
 * When copying the Device EUI from Console be sure lsb: is the byte order selected.
 */
//static const u1_t PROGMEM DEVEUI[8]= {0x00, 0x00, 0x00, 0xFE, 0xFF, 0x09, 0x1F, 0xAC};
//void os_getDevEui(u1_t *buf) {memcpy_P(buf, DEVEUI, 8);}

/*
 * The App EUI (APPEUI) must be in least-significant-byte order.
 * When copying the App EUI from Console be sure lsb: is the byte order selected.
 */
//static const u1_t PROGMEM APPEUI[8]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//void os_getArtEui(u1_t *buf) {memcpy_P(buf, APPEUI, 8);}

/*
 * The App Key (APPKEY) must be in most-significant-byte order.
 * When copying the App Key from Console be sure msb: is the byte order selected.
 */
//static const u1_t PROGMEM APPKEY[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//void os_getDevKey(u1_t *buf) {memcpy_P(buf, APPKEY, 16);}

// Set OTAA keys outside of the project in my case in the directory called OTAA_Keys where the Arduino sketchbooks are stored
//#include "../../../OTAA_Keys/RAK7200-OTAA-keys"
#include "../../../OTAA_Keys/RAK7200-36F0"


static uint8_t mydata[24] = "";
static uint8_t myDataSize = 0;
static osjob_t sendjob;

const unsigned TX_INTERVAL = 15; // Transmit every 15 seconds

const lmic_pinmap lmic_pins = {
        .nss = S76G_SX1276_SS,
        .rxtx = S76G_SX1276_ANTENNA_SWITCH_RXTX,
        .rst = S76G_SX1276_RESET,
        .dio = {S76G_SX1276_DIO0, S76G_SX1276_DIO1, S76G_SX1276_DIO2},
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 1000000
};

static NMEAGPS gps;
static gps_fix fix;
volatile byte blueLEDstate = HIGH; // Set LED state off
volatile byte redLEDstate = HIGH;
volatile byte greenLEDstate = HIGH;

static bool GNSS_probe() {
    unsigned long startTime = millis();
    char c1, c2;
    c1 = c2 = 0;

    GNSS_SERIAL.flush();
    while (millis() - startTime < 3000) {
        if (GNSS_SERIAL.available() > 0) {
            c1 = GNSS_SERIAL.read();
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

static void PPS_ISR(void) {
    CONSOLE_SERIAL.println("***  PPS_ISR  ***");
    greenLEDstate = !greenLEDstate;
    digitalWrite(S76G_RAK7200_GREEN_LED, greenLEDstate);
}

static void SerialPassThrough(void) {
    if (CONSOLE_SERIAL.available()) {
        GNSS_SERIAL.write(CONSOLE_SERIAL.read());
    }
    if (GNSS_SERIAL.available()) {
        CONSOLE_SERIAL.write(GNSS_SERIAL.read());
    }
}

static void scanI2Cbus(void) {
    byte err, addr;
    int nDevices = 0;

    CONSOLE_SERIAL.println("Scanning I2C bus");
    for (addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        err = Wire.endTransmission();
        if (err == 0) {
            CONSOLE_SERIAL.print("I2C device found at address 0x");
            if (addr < 16) {
                CONSOLE_SERIAL.print("0");
            }
            CONSOLE_SERIAL.println(addr, HEX);
            nDevices++;
        }
        else {
            if (err == 4) {
                CONSOLE_SERIAL.print("Unknown error at address 0x");
                if (addr < 16) {
                    CONSOLE_SERIAL.print("0");
                }
                CONSOLE_SERIAL.println(addr, HEX);
            }
        }
    }
    if (nDevices == 0) {
        CONSOLE_SERIAL.println("No I2C devices found\n");
    }
    CONSOLE_SERIAL.println("Scanning complete\n");
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16) {
        CONSOLE_SERIAL.print('0');
    }
    CONSOLE_SERIAL.print(v, HEX);
}

void onEvent(ev_t ev) {
    CONSOLE_SERIAL.print(os_getTime());
    CONSOLE_SERIAL.print(": ");
    switch (ev) {
        case EV_SCAN_TIMEOUT:
            CONSOLE_SERIAL.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            CONSOLE_SERIAL.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            CONSOLE_SERIAL.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            CONSOLE_SERIAL.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            CONSOLE_SERIAL.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            CONSOLE_SERIAL.println(F("EV_JOINED"));
            {
                u4_t netid = 0;
                devaddr_t devaddr = 0;
                u1_t nwkKey[16];
                u1_t artKey[16];
                LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                CONSOLE_SERIAL.print("netid: ");
                CONSOLE_SERIAL.println(netid, DEC);
                CONSOLE_SERIAL.print("devaddr: ");
                CONSOLE_SERIAL.println(__builtin_bswap32(devaddr), HEX);
                CONSOLE_SERIAL.print("AppSKey: ");
                for (size_t i = 0; i < sizeof(artKey); ++i) {
                    if (i != 0) {
                        CONSOLE_SERIAL.print("-");
                    }
                    printHex2(artKey[i]);
                }
                CONSOLE_SERIAL.println("");
                CONSOLE_SERIAL.print("NwkSKey: ");
                for (size_t i = 0; i < sizeof(nwkKey); ++i) {
                    if (i != 0) {
                        CONSOLE_SERIAL.print("-");
                    }
                    printHex2(nwkKey[i]);
                }
                CONSOLE_SERIAL.println();
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
            ||     CONSOLE_SERIAL.println(F("EV_RFU1"));
            ||     break;
            */
        case EV_JOIN_FAILED:
            CONSOLE_SERIAL.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            CONSOLE_SERIAL.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            CONSOLE_SERIAL.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK) {
                CONSOLE_SERIAL.println(F("Received ack"));
            }
            if (LMIC.dataLen) {
                CONSOLE_SERIAL.println(F("Received "));
                CONSOLE_SERIAL.println(LMIC.dataLen);
                CONSOLE_SERIAL.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            CONSOLE_SERIAL.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            CONSOLE_SERIAL.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            CONSOLE_SERIAL.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            CONSOLE_SERIAL.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            CONSOLE_SERIAL.println(F("EV_LINK_ALIVE"));
            break;
            /*
            || This event is defined but not used in the code. No
            || point in wasting codespace on it.
            ||
            || case EV_SCAN_FOUND:
            ||    CONSOLE_SERIAL.println(F("EV_SCAN_FOUND"));
            ||    break;
            */
        case EV_TXSTART:
            CONSOLE_SERIAL.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            CONSOLE_SERIAL.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            CONSOLE_SERIAL.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            CONSOLE_SERIAL.print(F("Unknown event: "));
            CONSOLE_SERIAL.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t *j) {
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        CONSOLE_SERIAL.println(F("OP_TXRXPEND, not sending"));
    }
    else {
        LMIC_setTxData2(1, mydata, myDataSize, 0);
        myDataSize = 0;
        CONSOLE_SERIAL.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    bool has_SX1276 = false;
    bool has_GNSS = false;
    time_t serialStart = 0;

    pinMode(S76G_RAK7200_BLUE_LED, OUTPUT);
    digitalWrite(S76G_RAK7200_BLUE_LED, blueLEDstate);
    pinMode(S76G_RAK7200_RED_LED, OUTPUT);
    digitalWrite(S76G_RAK7200_RED_LED, !redLEDstate);
    pinMode(S76G_RAK7200_GREEN_LED, OUTPUT);
    digitalWrite(S76G_RAK7200_GREEN_LED, greenLEDstate);
    pinMode(S76G_RAK7200_ADC_VBAT, INPUT);

    CONSOLE_SERIAL.begin(115200);
    serialStart = millis();
    while (!CONSOLE_SERIAL) {
        if ((millis() - serialStart) < 3000) {
            delay(100);
        }
        else {
            break;
        }
    }
    delay(3000);
    CONSOLE_SERIAL.println("\nS76G Power-on Self Test\n");
    CONSOLE_SERIAL.flush();

    Wire.setSCL(S76G_I2C1_SCL);
    Wire.setSDA(S76G_I2C1_SDA);
    Wire.begin();
    scanI2Cbus();
    Wire.end();

    SPI.setMISO(S76G_SX1276_MISO);
    SPI.setMOSI(S76G_SX1276_MOSI);
    SPI.setSCLK(S76G_SX1276_SCK);
    SPI.begin();

    digitalWrite(S76G_SX1276_SS, HIGH);
    pinMode(S76G_SX1276_SS, OUTPUT);

    digitalWrite(S76G_SX1276_RESET, HIGH);
    pinMode(S76G_SX1276_RESET, OUTPUT);

    // manually reset radio
    digitalWrite(S76G_SX1276_RESET, LOW);
    delay(5);
    digitalWrite(S76G_SX1276_RESET, HIGH);
    delay(5);

    digitalWrite(S76G_SX1276_SS, LOW);

    SPI.transfer(SX1276_RegVersion & 0x7F);
    has_SX1276 = (SPI.transfer(0x00) == 0x12 ? true : false);

    digitalWrite(S76G_SX1276_SS, HIGH);

    SPI.end();
    pinMode(S76G_SX1276_SS, INPUT);
    pinMode(S76G_SX1276_RESET, INPUT);

    CONSOLE_SERIAL.println("Built-in components:");

    CONSOLE_SERIAL.print("RADIO  - ");
    CONSOLE_SERIAL.println(has_SX1276 ? "PASS" : "FAIL");

    // power on GNSS
    pinMode(S76G_RAK7200_GNSS_POWER_ENABLE, OUTPUT);
    digitalWrite(S76G_RAK7200_GNSS_POWER_ENABLE, HIGH);
    delay(1200); // Delay 315µs to 800µs ramp up time

    // RAK7200 S76G 1PPS on GPIO PB5 (S76G_GNSS_1PPS)
    pinMode(S76G_GNSS_1PPS, INPUT);
    attachInterrupt(digitalPinToInterrupt(S76G_GNSS_1PPS), PPS_ISR, CHANGE);

    GNSS_SERIAL.begin(GNSS_BAUD_RATE);

    /* drive GNSS RST pin low */
    pinMode(S76G_GNSS_RESET_X, OUTPUT);
    digitalWrite(S76G_GNSS_RESET_X, LOW);

    /* activate 1.8V<->3.3V level shifters */
    pinMode(S76G_GNSS_LEVEL_SHIFTER, OUTPUT);
    digitalWrite(S76G_GNSS_LEVEL_SHIFTER, HIGH);

    /* keep RST low to ensure proper IC reset */
    delay(250);

    /* release */
    digitalWrite(S76G_GNSS_RESET_X, HIGH);

    /* give Sony GNSS few ms to warm up */
    delay(125);

    /* configure GNSS */
    GNSS_SERIAL.write("@GCD\r\n"); // Cold start
    delay(500);
    //GNSS_SERIAL.write("@GSW\r\n"); // Warm start
    //delay(250);
    //GNSS_SERIAL.write("@GSP\r\n"); // Hot start for position accuracy
    //delay(250);
    GNSS_SERIAL.write("@GPPS 0x1\r\n"); // Enable PPS
    delay(125);
    /*
     * @GNS Select the satellite systems to be used for positioning
     * bit 0 : GPS          0x01
     * bit 1 : GLONASS      0x02
     * bit 2 : SBAS         0x04
     * bit 3 : QZSS L1-CA   0x08
     * 
     */
    //GNSS_SERIAL.write("@GNS 0x7\r\n"); // Configure GPS, GLONASS, SBAS
    GNSS_SERIAL.write("@GNS 0x5\r\n"); // Configure GPS, SBAS
    //GNSS_SERIAL.write("@GNS 0x1\r\n"); // Configure GPS
    //GNSS_SERIAL.write("@GNS 0x2\r\n"); // Configure GLONASS
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
    //GNSS_SERIAL.write("@BSSL 0xFF\r\n"); // All NMEA sentences
    //GNSS_SERIAL.write("@BSSL 0xFE\r\n"); // All NMEA sentences but GGA
    //GNSS_SERIAL.write("@BSSL 0xB3\r\n"); // GGA, GLL, GNS, RMC, ZDA
    GNSS_SERIAL.write("@BSSL 0xA1\r\n"); // GGA, RMC, ZDA
    //GNSS_SERIAL.write("@BSSL 0x21\r\n"); // GGA and RMC
    delay(125);

    has_GNSS = GNSS_probe();
    CONSOLE_SERIAL.print("GNSS   - ");
    CONSOLE_SERIAL.println(has_GNSS ? "PASS" : "FAIL");

    CONSOLE_SERIAL.println();
    CONSOLE_SERIAL.println("POST is completed.\n\n");

    gpsPort.begin(115200);
    while (!gpsPort);
    CONSOLE_SERIAL.println("GNSS UART Initialized");

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
    CONSOLE_SERIAL.println("Radio Initialized");

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
        digitalWrite(S76G_RAK7200_GREEN_LED, greenLEDstate);
/*
        CONSOLE_SERIAL.print("Location: ");
        CONSOLE_SERIAL.print(fix.latitudeL());
        CONSOLE_SERIAL.print(',');
        CONSOLE_SERIAL.print(fix.longitudeL());
        if (fix.valid.altitude) {
            CONSOLE_SERIAL.print(", Altitude: ");
            CONSOLE_SERIAL.print(fix.altitude_cm());
        }
        CONSOLE_SERIAL.print(", Satellites: ");
        CONSOLE_SERIAL.print(fix.satellites);
        CONSOLE_SERIAL.print(", HDOP: ");
        CONSOLE_SERIAL.print(fix.hdop);
        CONSOLE_SERIAL.println();
*/
        // Prepare upstream data transmission at the next possible time.
        uint32_t i = 0;
        int32_t data = 0;
        data = (int32_t)(fix.latitudeL());
        CONSOLE_SERIAL.print("Location: ");
        CONSOLE_SERIAL.print(data);
        mydata[i++] = data >> 24;
        mydata[i++] = data >> 16;
        mydata[i++] = data >> 8;
        mydata[i++] = data;
        data = (int32_t)(fix.longitudeL());
        CONSOLE_SERIAL.print(", ");
        CONSOLE_SERIAL.print(data);
        mydata[i++] = data >> 24;
        mydata[i++] = data >> 16;
        mydata[i++] = data >> 8;
        mydata[i++] = data;
        data = (int32_t)(fix.altitude_cm());
        CONSOLE_SERIAL.print(", Altitude: ");
        CONSOLE_SERIAL.print(data);
        mydata[i++] = data >> 24;
        mydata[i++] = data >> 16;
        mydata[i++] = data >> 8;
        mydata[i++] = data;
        data = (uint32_t)(0);
        CONSOLE_SERIAL.print(", Accuracy: ");
        CONSOLE_SERIAL.print(data);
        mydata[i++] = data >> 24;
        mydata[i++] = data >> 16;
        mydata[i++] = data >> 8;
        mydata[i++] = data;
        data = (uint8_t)(fix.satellites);
        CONSOLE_SERIAL.print(", Satellites: ");
        CONSOLE_SERIAL.print(data);
        mydata[i++] = data;
        data = (uint16_t)(fix.hdop);
        CONSOLE_SERIAL.print(", HDOP: ");
        CONSOLE_SERIAL.print(data);
        mydata[i++] = data >> 8;
        mydata[i++] = data;
        data = (int32_t)((fix.speed_mph() * 100));
        //data = (int32_t)(0);
        CONSOLE_SERIAL.print(", Speed: ");
        CONSOLE_SERIAL.print(data);
        mydata[i++] = data >> 24;
        mydata[i++] = data >> 16;
        mydata[i++] = data >> 8;
        mydata[i++] = data;
        myDataSize = i;
        CONSOLE_SERIAL.print(float(analogRead(S76G_RAK7200_ADC_VBAT)) / 4096 * 3.30 / 0.6 * 10.0);
        CONSOLE_SERIAL.println();
    }
}
