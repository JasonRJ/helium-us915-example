/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network. It's pre-configured for the Adafruit
 * Feather M0 LoRa.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/
/*******************************************************************************
 *
 * Define the Region and Radio in
 * Arduino/libraries/MCCI_LoRaWAN_LMIC_library/project_config/lmic_project_config.h
 *
 * This is the library default configuration:
 * #define CFG_us915 1
 * #define CFG_sx1276_radio 1
 *
 * This is my configuration:
 * #define CFG_us915 1
 * #define CFG_sx1276_radio 1
 * #define LMIC_USE_INTERRUPTS
 * // Disable Class B features
 * #define DISABLE_PING
 * #define DISABLE_BEACONS
 *
 *******************************************************************************/

#include <lmic.h>     //http://librarymanager/All#MCCI+LoRaWAN+LMIC+library
#include <hal/hal.h>
#include <SPI.h>

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

// On the Helium network every Join, Downlink or Uplink LongFi (LoRaWAN) packet 0 to 24 bytes cost one Data Credit, 1DC = $0.00001
//static uint8_t LoRaPacketData[48] = "";
//static uint8_t LoRaPacketDataSize = 0;
static uint8_t LoRaPacketData[48] = "Hello World!";
static uint8_t LoRaPacketDataSize = 12;
static osjob_t sendjob;

// Schedule TX every TX_INTERVAL seconds (might become longer due to duty cycle limitations).
const unsigned TX_INTERVAL = 60;

// AcSiP S7xx Pin Mapping
const lmic_pinmap lmic_pins = {
        .nss = S7xx_SX127x_NSS,
        .rxtx = S7xx_SX127x_ANTENNA_SWITCH_RXTX,
        .rst = S7xx_SX127x_NRESET,
        .dio = {S7xx_SX127x_DIO0, S7xx_SX127x_DIO1, S7xx_SX127x_DIO2},
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 1000000
};

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
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, LoRaPacketData, LoRaPacketDataSize, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

    // Configure AcSiP S7xx Serial1 to Arduino Serial
    Serial.setTx(S7xx_CONSOLE_TX);
    Serial.setRx(S7xx_CONSOLE_RX);

    // Configure AcSiP S7xx SPI2 to Arduino SPI
    SPI.setMISO(S7xx_SX127x_MISO);
    SPI.setMOSI(S7xx_SX127x_MOSI);
    SPI.setSCLK(S7xx_SX127x_SCK);
    SPI.setSSEL(S7xx_SX127x_NSS);

    Serial.begin(115200);
    time_t serialStart = millis();
    while (!Serial) {
        if ((millis() - serialStart) < 3000) {
            delay(100);
        }
        else {
            break;
        }
    }
    delay(3000);
    Serial.println(F("Starting"));


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

    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
