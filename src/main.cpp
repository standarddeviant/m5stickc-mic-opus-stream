#include <Arduino.h>
#include <opus.h>

#define MIC_CHANNELS 1
#define MIC_BYTES_PER_SAMP 2
#define MIC_SAMPLE_RATE 16000
#define MIC_BLOCK_FRAMES 320 // 320 samples @ 16 kHz == 20 milliseconds
#define MIC_BUF_LEN
opus_int16 g_micBuf[MIC_CHANNELS * MIC_BLOCK_FRAMES] = { 0 };
unsigned char g_opBuf[MIC_CHANNELS * MIC_BLOCK_FRAMES * MIC_BYTES_PER_SAMP];
OpusEncoder *g_opEnc;

void test_opus_encoder_func( void * pvParameters ) {
    int ret;

    // create encoder
    Serial.printf("Calling :: g_opEnc = opus_encoder_create(MIC_SAMPLE_RATE, MIC_CHANNELS, OPUS_APPLICATION_AUDIO, &ret);\n");
    g_opEnc = opus_encoder_create(MIC_SAMPLE_RATE, MIC_CHANNELS, OPUS_APPLICATION_AUDIO, &ret);
    Serial.printf("g_opEnc = 0x%08X, ret = %d, opus_encoder_get_size(MIC_CHANNELS) = %d\n", 
                  (uint32_t)g_opEnc, ret, opus_encoder_get_size(1));

    // do a single encode call as a test
    Serial.printf("Calling :: ret = opus_encode(...)\n");
    ret = opus_encode(
        g_opEnc,
        (const opus_int16 *)(&( g_micBuf[0] )),
        MIC_BLOCK_FRAMES,
        (unsigned char *)(&( g_opBuf[0] )),
        sizeof(g_opBuf)
    );
    Serial.printf("ret = %d\n", ret);

    while(1) {
        Serial.printf("end of test_opus_encoder_func\n");
        delay(60000);
    }
} // end test_opus_encoder_func


void setup() {
    // put your setup code here, to run once:
    BaseType_t xReturned;
    TaskHandle_t xHandle = NULL;

    Serial.begin(115200);
    delay(1000);
    Serial.printf("starting test_opus_encoder_func task\n");

    /* Create task to test opus encoder */
    /* NOTE: we must run this in a created task with a large stack size;
             the default arduino task stack size is too small */
    xReturned = xTaskCreate(
        test_opus_encoder_func, /* Function that implements the task. */
        "test_opus_encoder",    /* Text name for the task. */
        32768,                  /* Stack size in words, not bytes. */
        ( void * ) 1,           /* Parameter passed into the task. */
        tskIDLE_PRIORITY,       /* Priority at which the task is created. */
        &xHandle                /* Used to pass out the created task's handle. */
    );

    Serial.printf("started test_opus_encoder_func with return code %d\n", xReturned);
}

void loop() {
  	// put your main code here, to run repeatedly:
	delay(1000);
}