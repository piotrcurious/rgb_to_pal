#include <driver/i2s.h>
#include <driver/timer.h>

#define SAMPLE_RATE     14745600   // fixed APLL rate
#define LINE_FREQ       15625
#define LINE_SAMPLES    (SAMPLE_RATE / LINE_FREQ) // ~943
#define PIXELS_PER_LINE 80
#define SAMPLES_PER_PIXEL ((LINE_SAMPLES-200)/PIXELS_PER_LINE)
#define SYNC_SAMPLES     60
#define BURST_SAMPLES    80
#define ACTIVE_START     (SYNC_SAMPLES + BURST_SAMPLES)
#define FRONT_PORCH      10

#define PIN_R 32
#define PIN_G 33
#define PIN_B 34
#define HSYNC_PIN 26
#define VSYNC_PIN 27
#define VIDEO_DAC_PIN 25

static uint16_t lineBuf[2][LINE_SAMPLES];
static volatile int bufIndex = 0;
static float prevV[PIXELS_PER_LINE] = {0};
static bool phaseAlt = false;
static int phaseIndex = 0;
static const int LUT_SIZE = 256;
static uint8_t lut[8][LUT_SIZE];

void IRAM_ATTR sampleRGB8(uint8_t &color) {
  // Read R/G/B from GPIO bits inline
  uint32_t in = GPIO.in;
  uint8_t r = (in >> PIN_R) & 1;
  uint8_t g = (in >> PIN_G) & 1;
  uint8_t b = (in >> PIN_B) & 1;
  color = (r<<2)|(g<<1)|b;
}

void buildLUT() {
  for (int c=0; c<8; c++) {
    float y = rgb2yuv[c].y;
    float u = rgb2yuv[c].u;
    float v = rgb2yuv[c].v;
    for (int ph=0; ph<LUT_SIZE; ph++) {
      float angle = 2*M_PI*ph/LUT_SIZE;
      lut[c][ph] = uint8_t(constrain(y + 0.6f*(u*cosf(angle)+(phaseAlt?-v:v)*sinf(angle)),0,1)*255);
    }
  }
}

void IRAM_ATTR onLineTimer(timer_event_t) {
  // Called every scanline
  while (gpio_get_level(HSYNC_PIN) == 1); // wait hsync low
  uint16_t *out = lineBuf[bufIndex];
  int idx=0;
  // sync
  for (; idx<SYNC_SAMPLES; idx++) out[idx]=0;
  // burst
  for (int i=0; i<BURST_SAMPLES; i++, idx++) {
    float burst = 0.3f + 0.2f * sinf(2*M_PI*(phaseIndex%LUT_SIZE)/LUT_SIZE)*(phaseAlt?-1:1);
    out[idx]=uint16_t(constrain(burst,0,1)*255);
    phaseIndex = (phaseIndex+1)&(LUT_SIZE-1);
  }
  // active video
  for (int px=0; px<PIXELS_PER_LINE; px++) {
    uint8_t col; sampleRGB8(col);
    float v = rgb2yuv[col].v;
    float vavg = 0.5f*(v + prevV[px]); prevV[px]=v;
    for (int s=0; s<SAMPLES_PER_PIXEL; s++, idx++) {
      out[idx] = lut[col][phaseIndex];
      phaseIndex = (phaseIndex+1)&(LUT_SIZE-1);
    }
  }
  // front porch
  for (; idx<LINE_SAMPLES; idx++) out[idx]=uint16_t(0.3f*255);

  i2s_push_sample((char*)out, LINE_SAMPLES*2, portMAX_DELAY);
  bufIndex ^= 1;
  phaseAlt = !phaseAlt;
}

void setup() {
  setupI2S_WITH_APLL();
  setupPinsAndInterrupts();
  buildLUT();
  timer_start(/* configured at 15.625kHz, callback onLineTimer */);
}

void loop() { /* idle, handled in ISR */ }
