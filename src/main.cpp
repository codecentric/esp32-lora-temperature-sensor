/*******
 * LoRaWAN Temperature Sensor
 * 
 * This example uses a TTGO LoRa32 board with a Semtech SX1276 LoRa transceiver
 * and an onboard OLED display to demonstrate how to connect a DHT22 temperature
 * and relative humidity sensor to a LoRaWAN network (The Things Network, for example).
 *  
 * Based on: https://github.com/mcci-catena/arduino-lmic/tree/master/examples/ttn-otaa-feather-us915-dht22
 */
#include <Arduino.h>

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "config.h"

// display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// DHT22 temperatur sensor
#include "DHT.h"

#define DHTPIN 13
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// LoRaWAN/lmic configuration
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t lora_payload[5];
static osjob_t lora_sendjob;
void do_send(osjob_t *j);

// Transmit interval in seconds; may be longer due to duty cycle limitations
const unsigned TX_INTERVAL_SECONDS = 30;

const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},
    .rxtx_rx_active = 0,
    .rssi_cal = 10,
    .spi_freq = 8000000,
};

static void print_as_hex(unsigned v)
{
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void display_show_text(int xpos, int line, const char *text)
{
    int16_t x;
    int16_t y;
    uint16_t w;
    uint16_t h;

    // clear area of the display first, then draw new text on the area
    display.getTextBounds(text, xpos * 8, line * 8, &x, &y, &w, &h);
    display.fillRect(x, y, w, h, 0);

    display.display();

    display.setCursor(xpos * 8, line * 8);
    display.println(text);
    display.display();
}

void display_show_int(int xpos, int line, int value, const char *prefix)
{
    char buffer[8];
    sprintf(buffer, "%s%u ", prefix, value);
    display_show_text(xpos, line, buffer);
}

void display_show_status(const char *text)
{
    display_show_text(0, 1, text);
}

void display_show_lora_stats(void)
{
    if (LMIC.adrEnabled)
    {
        display_show_text(9, 0, "ADR on ");
    }
    else
    {
        display_show_text(9, 0, "ADR off");
    }
    display_show_int(9, 1, LMIC.datarate, "DR ");

    int bandwidth = 0;
    int spreadfactor = 0;

    switch (LMIC.datarate)
    {
    case EU868_DR_SF7:
        bandwidth = 125;
        spreadfactor = 7;
        break;
    case EU868_DR_SF8:
        bandwidth = 125;
        spreadfactor = 8;
        break;
    case EU868_DR_SF9:
        bandwidth = 125;
        spreadfactor = 9;
        break;
    case EU868_DR_SF10:
        bandwidth = 125;
        spreadfactor = 10;
        break;
    case EU868_DR_SF11:
        bandwidth = 125;
        spreadfactor = 11;
        break;
    case EU868_DR_SF12:
        bandwidth = 125;
        spreadfactor = 12;
        break;
    default:
        break;
    }
    display_show_int(9, 2, spreadfactor, "SF ");
    display_show_int(9, 3, bandwidth, "BW ");
    display_show_int(9, 4, LMIC.seqnoUp, "TC ");

    // receive
    display_show_int(9, 5, (int)LMIC.rssi, "R ");
    display_show_int(9, 6, (int)LMIC.snr, "S ");
    display_show_int(9, 7, LMIC.seqnoDn, "RC ");
}

void onEvent(ev_t ev)
{
    Serial.print(os_getTime());
    Serial.print(": ");
    switch (ev)
    {
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
            Serial.println(devaddr, HEX);
            Serial.print("AppSKey: ");
            for (size_t i = 0; i < sizeof(artKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                print_as_hex(artKey[i]);
            }
            Serial.println("");
            Serial.print("NwkSKey: ");
            for (size_t i = 0; i < sizeof(nwkKey); ++i)
            {
                if (i != 0)
                    Serial.print("-");
                print_as_hex(nwkKey[i]);
            }
            Serial.println();
        }

        // enable ADR and set DR to 5 after joining
        LMIC_setAdrMode(1);
        LMIC_setDrTxpow(EU868_DR_SF7, 14);

        break;
    case EV_JOIN_FAILED:
        Serial.println(F("EV_JOIN_FAILED"));
        break;
    case EV_REJOIN_FAILED:
        Serial.println(F("EV_REJOIN_FAILED"));
        break;
    case EV_TXCOMPLETE:
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        display_show_status("TX done ");

        if (LMIC.txrxFlags & TXRX_ACK)
            Serial.println(F("Received ack"));
        if (LMIC.dataLen)
        {
            Serial.println(F("Received "));
            Serial.println(LMIC.dataLen);
            Serial.println(F(" bytes of payload"));
        }

        display_show_lora_stats();

        // Schedule next transmission
        os_setTimedCallback(&lora_sendjob, os_getTime() + sec2osticks(TX_INTERVAL_SECONDS), do_send);
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
        Serial.println((unsigned)ev);
        break;
    }
}

void do_send(osjob_t *j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND)
    {
        Serial.println(F("OP_TXRXPEND, not sending"));
    }
    else
    {
        display_show_status("TX start");
        float temperature = dht.readTemperature();
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" *C");

        float relative_humidity = dht.readHumidity();
        Serial.print("Rel. humidity: ");
        Serial.print(relative_humidity);
        Serial.println(" %");

        char temperature_buffer[5];
        char humidity_buffer[5];
        char temperature_display_buffer[8] = "T: ";
        char humidity_display_buffer[8] = "H: ";

        // display temperature and humidity on display
        dtostrf(temperature, 2, 1, temperature_buffer);
        strcat(temperature_display_buffer, temperature_buffer);
        dtostrf(relative_humidity, 2, 1, humidity_buffer);
        strcat(humidity_display_buffer, humidity_buffer);

        display_show_text(0, 2, temperature_display_buffer);
        display_show_text(0, 3, humidity_display_buffer);

        // prepare message to be sent
        uint16_t payload_temperature = LMIC_f2sflt16(temperature / 100);
        lora_payload[0] = lowByte(payload_temperature);
        lora_payload[1] = highByte(payload_temperature);

        uint16_t payload_humidity = LMIC_f2sflt16(relative_humidity / 100);
        lora_payload[2] = lowByte(payload_humidity);
        lora_payload[3] = highByte(payload_humidity);

        // schedule transmission of the message on port 1 and do not request an ack
        lmic_tx_error_t txResult = LMIC_setTxData2(1, lora_payload, sizeof(lora_payload) - 1, 0);

        if (txResult == LMIC_ERROR_SUCCESS)
        {
            display_show_lora_stats();
        }
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup()
{
    delay(100);
    while (!Serial)
        ;

    Serial.begin(115000);
    Serial.println(F("Starting"));

    dht.begin();

    Wire.begin(OLED_SDA, OLED_SCL);

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);

    display_show_text(0, 0, "LoRa");
    display_show_status("init...");

    display.display();

    // LMIC init
    os_init();
    Serial.println("init done, resetting lmic");
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    Serial.println("reset done, set link mode");
    // Enable ADR mode
    LMIC_setAdrMode(1);

    Serial.println("ADR mode disabled, set dr and tx power");
    LMIC_setDrTxpow(EU868_DR_SF7, 14);

    Serial.println("starting job...");
    // Start job (sending automatically starts OTAA too)
    do_send(&lora_sendjob);
}

void loop()
{
    // we call the LMIC's runloop processor. This will cause things to happen based on events and time. One
    // of the things that will happen is callbacks for transmission complete or received messages. We also
    // use this loop to queue periodic data transmissions.  You can put other things here in the `loop()` routine,
    // but beware that LoRaWAN timing is pretty tight, so if you do more than a few milliseconds of work, you
    // will want to call `os_runloop_once()` every so often, to keep the radio running.
    os_runloop_once();
}