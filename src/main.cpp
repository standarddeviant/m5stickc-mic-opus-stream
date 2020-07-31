#include <Arduino.h>
#include <M5StickC.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2s.h>
#include <freertos/ringbuf.h>
#include <opus.h>
#include "WIFI_SECRETS.h"
// if you don't have WIFI_SECRETS.H, fill in and uncomment the following lines
// and copy it to WIFI_SECRETS.H in the same folder as main.cpp
// const char* ssid = "WIFI_SSID_FROM_WIFI_SECRETS.H";
// const char* password = "WIFI_PASSWORD_FROM_WIFI_SECRETS.H";

#define MIC_PIN_CLK  0
#define MIC_PIN_DATA 34
#define READ_LEN (2 * 256)

#define MIC_CHANNELS 1
#define MIC_BYTES_PER_SAMP 2
#define MIC_SAMPLE_RATE 48000
#define MIC_BUF_FRAMES 960 // 320 samples @ 16 kHz == 20 milliseconds
#define MIC_BUF_LEN (MIC_CHANNELS * MIC_BUF_FRAMES) // units of opus_int16
#define MIC_BUF_BYTES (MIC_BUF_LEN * MIC_BYTES_PER_SAMP)
opus_int16 g_micBuf[MIC_CHANNELS * MIC_BUF_FRAMES] = { 0 };
opus_int16 g_micBufMin = (opus_int16)32767, g_micBufMax = (opus_int16)-32768;

#define OP_BUF_BYTES (MIC_CHANNELS * MIC_BUF_FRAMES * MIC_BYTES_PER_SAMP)
#define OP_RINGBUF_ITEM_SIZE OP_BUF_BYTES
unsigned char g_opBuf[OP_BUF_BYTES];
RingbufHandle_t g_opRingbuf;
uint32_t g_sendCount = 0, g_receiveCount = 0, g_maxItemSize = 0;
OpusEncoder *g_opEnc;


#define WS_SERVER_PORT 42042
#define WS_SERVER_HOSTNAME "m5c-mic-opus"
WebSocketsServer webSocket = WebSocketsServer(WS_SERVER_PORT);
StaticJsonDocument<4000> g_doc;
#define MPK_MAX_SZ 4000
char g_mpkBuf[MPK_MAX_SZ];
size_t g_mpkSize;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            {
                Serial.print("WS: We lost one!\n");
            }
            break;
        case WStype_CONNECTED:
            {
                Serial.print("WS: We got one!\n");
				// webSocket.sendTXT(num, "Connected");
                // TODO - send opus header on connect?
                // webSocket.sendBIN(num, ???)
            }
            break;
        case WStype_ERROR:
        case WStype_TEXT:
        case WStype_BIN:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
        case WStype_PING:
        case WStype_PONG:
            break;
    }
}

void ringbufInit() {
    g_opRingbuf = xRingbufferCreateNoSplit(OP_RINGBUF_ITEM_SIZE, 2);
    if(NULL == g_opRingbuf) {
        Serial.printf("ERR: g_opRingbuf == NULL\n");
    }
}


void opusEncoderInit(void) {
    int ret;
    // create encoder
    Serial.printf("Calling :: g_opEnc = opus_encoder_create(MIC_SAMPLE_RATE, MIC_CHANNELS, OPUS_APPLICATION_AUDIO, &ret);\n");
    g_opEnc = opus_encoder_create(MIC_SAMPLE_RATE, MIC_CHANNELS, OPUS_APPLICATION_AUDIO, &ret);
    Serial.printf("g_opEnc = 0x%08X, ret = %d, opus_encoder_get_size(MIC_CHANNELS) = %d\n", 
                  (uint32_t)g_opEnc, ret, opus_encoder_get_size(1));
}


void micI2Sinit(void) {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
        .sample_rate =  MIC_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, // is fixed at 12bit, stereo, MSB
        .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 2,
        .dma_buf_len = MIC_BUF_FRAMES,
    };
    i2s_pin_config_t pin_config = {
        .bck_io_num   = I2S_PIN_NO_CHANGE,
        .ws_io_num    = MIC_PIN_CLK,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num  = MIC_PIN_DATA,
    };

    // initialize i2s driver
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_set_clk(I2S_NUM_0, MIC_SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}


void micOpusTaskFunc( void * pvParameters ) {
    int ret;
    size_t bytesRead, itemSize;

    opusEncoderInit(); // init step 2

    micI2Sinit();      // init step 3

    while(1) {
        // read i2s 
        ret = i2s_read(I2S_NUM_0, (void *)g_micBuf, MIC_BUF_BYTES, &bytesRead, pdMS_TO_TICKS(1000));
        if(ret != ESP_OK) {
            Serial.printf("ERR: i2s_read = %d\n", ret);
            continue;
        }

        for(int ix = 0; ix < MIC_BUF_LEN; ix++) {
            if(g_micBuf[ix] < g_micBufMin) g_micBufMin = g_micBuf[ix];
            if(g_micBuf[ix] > g_micBufMax) g_micBufMax = g_micBuf[ix];
        }

        // encode audio from g_micBuf to g_opBuf
        ret = opus_encode(
            g_opEnc,
            (const opus_int16 *)(&( g_micBuf[0] )),
            MIC_BUF_FRAMES,
            (unsigned char *)(&( g_opBuf[0] )),
            sizeof(g_opBuf)
        );

        // TODO - check value of ret
        // check_opus_encode_return_value();

        // put opBuf on to g_opRingbuf
        itemSize = ret;
        ret = xRingbufferSend(g_opRingbuf, (const void *)g_opBuf, itemSize, pdMS_TO_TICKS(1000));
        g_sendCount++;
        if(itemSize > g_maxItemSize) {
            g_maxItemSize = itemSize;
        }

    } // end while(1)
} // end test_opus_encoder_func


void wifiTaskFunc( void * pvParameters ) {
    int ix, ret;     // loop and return vars
    void *item;      // to receive from ringbuf
    size_t itemSize; // to receive from ringbuf
    uint8_t *u8ptr;  // to traverse ringbuf item

    // setup wifi code here

    // Basic M5Stick-C Init
    M5.begin();
    M5.Axp.ScreenBreath(8); // turn the screen 0=off, 8=dim
    M5.Lcd.setRotation(3);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("Connecting to WiFi...");

    // WiFi Init
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED) {
        delay(100);
    }

    // WebSocket Init
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    // DNS Init
    if(!MDNS.begin(WS_SERVER_HOSTNAME)) {
        Serial.println("Error starting mDNS");
    }
    MDNS.addService("ws", "tcp", WS_SERVER_PORT);

    IPAddress ip = WiFi.localIP();
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf("Connect to\n%s:%d",
        WS_SERVER_HOSTNAME, WS_SERVER_PORT);

    while(1) {
        item = xRingbufferReceive(g_opRingbuf, &itemSize, pdMS_TO_TICKS(1000));
        if(NULL == item) {
            Serial.printf("xRingbufferReceive return'ed NULL\n");
            continue;
        }
        g_receiveCount++;

        // pack opus bytes in to a msgpack buffer
        g_doc.clear();
        u8ptr = (uint8_t *)item;
        JsonArray _op = g_doc.createNestedArray("opus");
        for(ix = 0; ix < itemSize; ix++)
            _op.add(*(u8ptr++));
        g_mpkSize = serializeMsgPack(g_doc, &(g_mpkBuf[0]), sizeof(g_mpkBuf));

        // after packing opus bytes in to msgpack buffer,
        // "return" the "item" to the ringbuf
        vRingbufferReturnItem(g_opRingbuf, item);

        // broadcast msgpack containing opus encoded bytes via websocket
        ret = webSocket.broadcastBIN((uint8_t *) &(g_mpkBuf[0]), g_mpkSize);

        // TODO - check return variable, ret

    } // end while(1)

} // end void wifiTaskFunc( void * pvParameters )


void setup() {
    // put your setup code here, to run once:
    BaseType_t xReturned;
    TaskHandle_t xHandle = NULL;

    // set up serial to see debug, warning, error messages, etc.
    Serial.begin(115200); delay(10);

    // set up the ring-buffer to communicate between micOpusTask and wifiTask
    // NOTE: this must be done before starting micOpusTask and wifiTask
    ringbufInit();

    Serial.printf("starting wifiTask\n");
    /* Create task to test opus encoder */
    /* NOTE: we must run this in a created task with a large stack size;
             the default arduino task stack size is too small */
    xReturned = xTaskCreate(
        wifiTaskFunc,       /* Function that implements the task. */
        "wifiTask",         /* Text name for the task. */
        8192,               /* Stack size in words, not bytes. */
        ( void * ) 1,       /* Parameter passed into the task. */
        tskIDLE_PRIORITY+1, /* Priority at which the task is created. */
        &xHandle            /* Used to pass out the created task's handle. */
    );
    Serial.printf("started wifiTask with return code %d\n", xReturned);


    Serial.printf("starting test_opus_encoder_func task\n");
    /* Create task to test opus encoder */
    /* NOTE: we must run this in a created task with a large stack size;
             the default arduino task stack size is too small */
    xReturned = xTaskCreate(
        micOpusTaskFunc, /* Function that implements the task. */
        "micOpusTask",    /* Text name for the task. */
        32768,              /* Stack size in words, not bytes. */
        ( void * ) 1,       /* Parameter passed into the task. */
        tskIDLE_PRIORITY+1, /* Priority at which the task is created. */
        &xHandle            /* Used to pass out the created task's handle. */
    );
    Serial.printf("started micOpusTask with return code %d\n", xReturned);

    while(1) {
        delay(2000);
        Serial.printf("MAXSZ = %d, MINVAL = %d, MAXVAL = %d, SEND = %d, RECEIVE = %d\n",
                      g_maxItemSize, g_micBufMin, g_micBufMax, g_sendCount, g_receiveCount);
        g_maxItemSize = 0;
        g_micBufMin = (opus_int16) 32767;
        g_micBufMax = (opus_int16)-32768;
    }
}

void loop() {
  	// put your main code here, to run repeatedly:
	delay(1000);
}