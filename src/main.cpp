#include <Arduino.h>
#include <M5StickC.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/i2s.h>
#include <freertos/ringbuf.h>
#include <opus.h>
// #include "WIFI_SECRETS.h"

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


#define OP_BUF_BYTES (MIC_CHANNELS * MIC_BUF_FRAMES * MIC_BYTES_PER_SAMP)
#define OP_RINGBUF_ITEM_SIZE OP_BUF_BYTES
unsigned char g_opBuf[OP_BUF_BYTES];
RingbufHandle_t g_opRingbuf;
uint32_t g_sendCount = 0, g_receiveCount = 0, g_maxItemSize = 0;
OpusEncoder *g_opEnc;


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
    void *item;
    size_t itemSize;

    // setup wifi code here

    while(1) {
        item = xRingbufferReceive(g_opRingbuf, &itemSize, pdMS_TO_TICKS(1000));
        if(NULL == item) {
            Serial.printf("xRingbufferReceive return'ed NULL\n");
            continue;
        }
        g_receiveCount++;
        vRingbufferReturnItem(g_opRingbuf, item);
    }
}


void setup() {
    // put your setup code here, to run once:
    BaseType_t xReturned;
    TaskHandle_t xHandle = NULL;

    Serial.begin(115200); delay(10);

    ringbufInit();     // init step 1

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
        delay(1000);
        Serial.printf("MAXSZ = %d, SEND = %d, RECEIVE = %d\n",
                      g_maxItemSize, g_sendCount, g_receiveCount);
        g_maxItemSize = 0;
    }
}

void loop() {
  	// put your main code here, to run repeatedly:
	delay(1000);
}