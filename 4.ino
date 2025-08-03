// ESP32 PAL composite video encoder with 8-color RGB input and full chroma modulation // Author: ChatGPT

#include <driver/i2s.h> #include <driver/timer.h> #include <soc/gpio_struct.h> #include <math.h>

#define SAMPLE_RATE     14745600 #define LINE_FREQ       15625 #define LINE_SAMPLES    (SAMPLE_RATE / LINE_FREQ) #define PIXELS_PER_LINE 80 #define SAMPLES_PER_PIXEL ((LINE_SAMPLES - 200) / PIXELS_PER_LINE) #define SYNC_SAMPLES     60 #define BURST_SAMPLES    80 #define ACTIVE_START     (SYNC_SAMPLES + BURST_SAMPLES) #define FRONT_PORCH      10

#define PIN_R 32 #define PIN_G 33 #define PIN_B 34 #define HSYNC_PIN 26 #define VSYNC_PIN 27 #define VIDEO_DAC_PIN 25

static uint16_t lineBuf[2][LINE_SAMPLES]; static volatile int bufIndex = 0; static float prevV[PIXELS_PER_LINE] = {0}; static bool phaseAlt = false; static int phaseIndex = 0; static const int LUT_SIZE = 256; static uint8_t lut[8][LUT_SIZE];

struct YUV { float y, u, v; }; const YUV rgb2yuv[8] = { {0.0,  0.0,  0.0}, // black {0.3,  0.6,  0.6}, // blue {0.6, -0.6,  0.6}, // red {0.5,  0.0,  1.0}, // magenta {0.7,  0.6, -0.6}, // green {0.6,  1.0,  0.0}, // cyan {0.8,  0.0,  0.6}, // yellow {1.0,  0.0,  0.0}  // white };

void IRAM_ATTR sampleRGB8(uint8_t &color) { uint32_t in = GPIO.in; uint8_t r = (in >> PIN_R) & 1; uint8_t g = (in >> PIN_G) & 1; uint8_t b = (in >> PIN_B) & 1; color = (r << 2) | (g << 1) | b; }

void buildLUT() { for (int c = 0; c < 8; c++) { float y = rgb2yuv[c].y; float u = rgb2yuv[c].u; float v = rgb2yuv[c].v; for (int ph = 0; ph < LUT_SIZE; ph++) { float angle = 2 * M_PI * ph / LUT_SIZE; float chroma = y + 0.6f * (u * cosf(angle) + (phaseAlt ? -v : v) * sinf(angle)); lut[c][ph] = uint8_t(fminf(fmaxf(chroma, 0.0f), 1.0f) * 255); } } }

void IRAM_ATTR onLineTimer(void *arg) { while (gpio_get_level(HSYNC_PIN) == 1); // wait for hsync low uint16_t *out = lineBuf[bufIndex]; int idx = 0;

for (; idx < SYNC_SAMPLES; idx++) out[idx] = 0; for (int i = 0; i < BURST_SAMPLES; i++, idx++) { float burst = 0.3f + 0.2f * sinf(2 * M_PI * (phaseIndex % LUT_SIZE) / LUT_SIZE) * (phaseAlt ? -1 : 1); out[idx] = uint16_t(fminf(fmaxf(burst, 0.0f), 1.0f) * 255); phaseIndex = (phaseIndex + 1) & (LUT_SIZE - 1); }

for (int px = 0; px < PIXELS_PER_LINE; px++) { uint8_t col; sampleRGB8(col); float v = rgb2yuv[col].v; float vavg = 0.5f * (v + prevV[px]); prevV[px] = v; for (int s = 0; s < SAMPLES_PER_PIXEL; s++, idx++) { out[idx] = lut[col][phaseIndex]; phaseIndex = (phaseIndex + 1) & (LUT_SIZE - 1); } }

for (; idx < LINE_SAMPLES; idx++) out[idx] = uint16_t(0.3f * 255); i2s_write_bytes(I2S_NUM_0, (char*)out, LINE_SAMPLES * 2, portMAX_DELAY); bufIndex ^= 1; phaseAlt = !phaseAlt; }

void setupI2S() { i2s_config_t config = { .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), .sample_rate = SAMPLE_RATE, .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, .communication_format = I2S_COMM_FORMAT_I2S_MSB, .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, .dma_buf_count = 4, .dma_buf_len = LINE_SAMPLES / 2, .use_apll = true, .tx_desc_auto_clear = true, .fixed_mclk = 0 }; i2s_driver_install(I2S_NUM_0, &config, 0, NULL); i2s_set_pin(I2S_NUM_0, NULL); gpio_set_direction(VIDEO_DAC_PIN, GPIO_MODE_OUTPUT); }

void setupTimer() { timer_config_t timer = { .divider = 80, .counter_dir = TIMER_COUNT_UP, .counter_en = TIMER_PAUSE, .alarm_en = TIMER_ALARM_EN, .auto_reload = true }; timer_init(TIMER_GROUP_0, TIMER_0, &timer); timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 80000000 / LINE_FREQ); timer_enable_intr(TIMER_GROUP_0, TIMER_0); timer_isr_register(TIMER_GROUP_0, TIMER_0, onLineTimer, NULL, ESP_INTR_FLAG_IRAM, NULL); timer_start(TIMER_GROUP_0, TIMER_0); }

void setup() { pinMode(PIN_R, INPUT); pinMode(PIN_G, INPUT); pinMode(PIN_B, INPUT); pinMode(HSYNC_PIN, INPUT); pinMode(VSYNC_PIN, INPUT); pinMode(VIDEO_DAC_PIN, OUTPUT);

buildLUT(); setupI2S(); setupTimer(); }

void loop() { delay(100); }

