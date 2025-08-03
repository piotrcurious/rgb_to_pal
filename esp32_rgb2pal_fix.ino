#include <driver/i2s.h>
#include <math.h>

#define SAMPLE_RATE     14745600  // 14.7456 MHz (3.33 × PAL subcarrier)
#define LINE_FREQ       15625     // PAL line rate
#define LINE_SAMPLES    (SAMPLE_RATE / LINE_FREQ)  // ≈ 943 samples/line

#define PIN_R           32
#define PIN_G           33
#define PIN_B           34
#define PIN_HSYNC       26
#define PIN_VSYNC       27
#define VIDEO_DAC_PIN   25  // DAC1

#define PIXELS_PER_LINE   80
#define SYNC_PULSE_SAMPLES  60
#define BURST_LENGTH         80
#define ACTIVE_START         160
#define FRONT_PORCH_SAMPLES  10

uint16_t lineBuffer[LINE_SAMPLES];
float prev_v_line[PIXELS_PER_LINE] = {0};

// YUV definition for 8 colors
struct Color {
  float y, u, v;
};
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

float chroma_inc = 2.0f * PI * 4433618.75f / SAMPLE_RATE;  // subcarrier delta phase

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

void buildPALLine(bool phaseAlternate) {
  float phase = 0.0f;
  int sample = 0;

  // 1. SYNC
  for (; sample < SYNC_PULSE_SAMPLES; ++sample)
    lineBuffer[sample] = 0;

  // 2. Burst on back porch
  for (int i = 0; i < BURST_LENGTH; ++i) {
    float burst = 0.3f + 0.2f * sinf(phase) * (phaseAlternate ? -1.0f : 1.0f);
    lineBuffer[sample++] = (uint16_t)(constrain(burst, 0.0f, 1.0f) * 255.0f);
    phase += chroma_inc;
  }

  // 3. Active video: read RGB → YUV → modulate chroma
  int samplesPerPixel = (LINE_SAMPLES - sample - FRONT_PORCH_SAMPLES) / PIXELS_PER_LINE;

  for (int px = 0; px < PIXELS_PER_LINE; ++px) {
    uint8_t rgb = readRGB();
    Color c = rgb2yuv[rgb];

    float v_phase = c.v;
    float v_avg = (v_phase + prev_v_line[px]) * 0.5f;
    prev_v_line[px] = v_phase;

    float v_used = phaseAlternate ? -v_avg : v_avg;

    for (int s = 0; s < samplesPerPixel; ++s) {
      float chroma = c.u * cosf(phase) + v_used * sinf(phase);
      float signal = c.y + 0.6f * chroma;
      lineBuffer[sample++] = (uint16_t)(constrain(signal, 0.0f, 1.0f) * 255.0f);
      phase += chroma_inc;
    }
  }

  // 4. Front porch
  for (; sample < LINE_SAMPLES; ++sample)
    lineBuffer[sample] = (uint16_t)(0.3f * 255.0f);  // black level
}

void setup() {
  Serial.begin(115200);
  setupI2S();
  setupPins();
  Serial.println("ESP32 PAL RGB encoder initialized.");
}

void loop() {
  waitForVSYNC();  // Sync on frame

  for (int line = 0; line < 312; ++line) {
    waitForHSYNC();                     // Wait for line trigger
    buildPALLine(line % 2 == 1);        // Alternate phase per line
    size_t written;
    i2s_write(I2S_NUM_0, (char*)lineBuffer, sizeof(lineBuffer), &written, portMAX_DELAY);
  }
}
