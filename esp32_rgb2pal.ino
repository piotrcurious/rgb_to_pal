#include <driver/i2s.h>
#include <math.h>

#define SAMPLE_RATE   14745600  // 14.7456 MHz for 3.33x PAL subcarrier
#define LINE_FREQ     15625     // 15.625 kHz PAL line rate
#define LINE_SAMPLES  (SAMPLE_RATE / LINE_FREQ)  // ≈ 943 samples per line

#define VIDEO_PIN     25  // DAC1
#define PIN_R         32
#define PIN_G         33
#define PIN_B         34
#define PIN_HSYNC     26
#define PIN_VSYNC     27

#define PIXELS_PER_LINE   80
#define ACTIVE_START      160  // sample index for active video
#define FRONT_PORCH_SAMPLES 60
#define BURST_LENGTH_SAMPLES 80
#define SYNC_PULSE_SAMPLES 60

uint16_t lineBuffer[LINE_SAMPLES];

float subcarrier_phase_table[LINE_SAMPLES];
float burst_waveform[BURST_LENGTH_SAMPLES];

struct Color { float y, u, v; };

// RGB3 → YUV values
Color rgb2yuv[8] = {
  {0.00,  0.00,  0.00}, // black
  {0.11,  0.19, -0.32}, // blue
  {0.59, -0.28,  0.11}, // green
  {0.70, -0.09, -0.21}, // cyan
  {0.30,  0.30,  0.30}, // red
  {0.41,  0.49, -0.02}, // magenta
  {0.89,  0.02,  0.41}, // yellow
  {1.00,  0.00,  0.00}  // white
};

void setupI2S() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 1024,
    .use_apll = false
  };
  i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL);
  i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
}

void setupPins() {
  pinMode(PIN_R, INPUT);
  pinMode(PIN_G, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(PIN_HSYNC, INPUT);
  pinMode(PIN_VSYNC, INPUT);
}

void setupBurstAndCarrier() {
  for (int i = 0; i < LINE_SAMPLES; ++i) {
    float t = (float)i / SAMPLE_RATE;
    subcarrier_phase_table[i] = sinf(2.0f * PI * 4433618.75f * t);
  }
  for (int i = 0; i < BURST_LENGTH_SAMPLES; ++i) {
    float t = (float)i / SAMPLE_RATE;
    burst_waveform[i] = sinf(2.0f * PI * 4433618.75f * t);
  }
}

uint8_t readRGB() {
  return (digitalRead(PIN_R) << 2) | (digitalRead(PIN_G) << 1) | digitalRead(PIN_B);
}

void waitForVSYNC() {
  while (digitalRead(PIN_VSYNC) == HIGH);
  while (digitalRead(PIN_VSYNC) == LOW);
}

void waitForHSYNC() {
  while (digitalRead(PIN_HSYNC) == HIGH);
  while (digitalRead(PIN_HSYNC) == LOW);
}

void buildLineFromRGB(bool phaseAlternate) {
  // 1. Sync pulse
  for (int i = 0; i < SYNC_PULSE_SAMPLES; ++i)
    lineBuffer[i] = 0;

  // 2. Back porch with color burst
  for (int i = 0; i < BURST_LENGTH_SAMPLES; ++i) {
    float val = 0.3f + 0.2f * burst_waveform[i] * (phaseAlternate ? -1.0f : 1.0f);
    lineBuffer[SYNC_PULSE_SAMPLES + i] = (uint16_t)(val * 255.0f);
  }

  // 3. Active video from RGB inputs
  int sample = ACTIVE_START;
  int samplesPerPixel = (LINE_SAMPLES - ACTIVE_START - 10) / PIXELS_PER_LINE;

  for (int px = 0; px < PIXELS_PER_LINE; ++px) {
    uint8_t rgb = readRGB();
    Color c = rgb2yuv[rgb];

    for (int s = 0; s < samplesPerPixel; ++s) {
      float carrier = subcarrier_phase_table[sample];
      float chroma = c.u * cosf(2 * PI * 4433618.75f * sample / SAMPLE_RATE)
                   + (phaseAlternate ? -1 : 1) * c.v * sinf(2 * PI * 4433618.75f * sample / SAMPLE_RATE);
      float val = c.y + 0.6f * chroma;
      val = constrain(val, 0.0f, 1.0f);
      lineBuffer[sample++] = (uint16_t)(val * 255.0f);
    }
  }

  // 4. Front porch
  for (; sample < LINE_SAMPLES; ++sample)
    lineBuffer[sample] = 64;
}

void setup() {
  Serial.begin(115200);
  setupI2S();
  setupPins();
  setupBurstAndCarrier();
  Serial.println("PAL encoder with RGB input initialized.");
}

void loop() {
  waitForVSYNC();  // start of frame

  for (int line = 0; line < 312; ++line) {
    waitForHSYNC();  // wait for next scanline
    buildLineFromRGB(line % 2);  // PAL alternates chroma phase
    size_t bytes_written;
    i2s_write(I2S_NUM_0, (const char*)lineBuffer, sizeof(lineBuffer), &bytes_written, portMAX_DELAY);
  }
}
