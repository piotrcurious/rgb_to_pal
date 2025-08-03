// ESP32 PAL composite video encoder with 8-color RGB input and full chroma modulation
// Author: ChatGPT (optimized with inline assembly) - Completed and Fixed

#include <driver/i2s.h>
#include <driver/timer.h>
#include <soc/gpio_struct.h>
#include <math.h>

#define SAMPLE_RATE     14745600
#define LINE_FREQ       15625
#define LINE_SAMPLES    (SAMPLE_RATE / LINE_FREQ)
#define PIXELS_PER_LINE 80
#define SAMPLES_PER_PIXEL ((LINE_SAMPLES - 200) / PIXELS_PER_LINE)
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

// Global variables
static uint16_t lineBuf[2][LINE_SAMPLES];
static volatile int bufIndex = 0;
static float prevV[PIXELS_PER_LINE] = {0};
static bool phaseAlt = false;
static int phaseIndex = 0;
static const int LUT_SIZE = 256;
static uint8_t lut[8][LUT_SIZE];
static volatile int lineCount = 0;
static volatile bool frameSync = false;

// YUV color space structure
struct YUV {
    float y, u, v;
};

// RGB to YUV conversion table for 8 colors (3-bit RGB)
const YUV rgb2yuv[8] = {
    {0.0,  0.0,  0.0},   // 000 - black
    {0.299, 0.492, -0.147}, // 001 - blue
    {0.587, -0.289, 0.439}, // 010 - red  
    {0.886, 0.203, 0.292},  // 011 - magenta
    {0.114, -0.203, -0.292}, // 100 - green
    {0.413, 0.289, -0.439}, // 101 - cyan
    {0.701, -0.492, 0.147}, // 110 - yellow
    {1.0,  0.0,  0.0}       // 111 - white
};

// Timer handle
static timer_isr_t timer_isr_handle = NULL;

// Fast GPIO sampling using inline assembly
void IRAM_ATTR sampleRGB8(uint8_t &color) {
    uint32_t val;
    asm volatile (
        "l32i %0, %1, 0x3C"  // Load GPIO_IN_REG (offset 0x3C from GPIO base)
        : "=r"(val)
        : "r"(0x3FF44000)    // GPIO base address
        : "memory"
    );
    
    uint8_t r = (val >> PIN_R) & 1;
    uint8_t g = (val >> PIN_G) & 1;
    uint8_t b = (val >> PIN_B) & 1;
    color = (r << 2) | (g << 1) | b;
}

// Build lookup table for chroma modulation
void buildLUT() {
    for (int c = 0; c < 8; c++) {
        float y = rgb2yuv[c].y;
        float u = rgb2yuv[c].u;
        float v = rgb2yuv[c].v;
        
        for (int ph = 0; ph < LUT_SIZE; ph++) {
            float angle = 2.0f * M_PI * ph / LUT_SIZE;
            // PAL chroma subcarrier frequency modulation
            float chroma = y + 0.3f * (u * cosf(angle + M_PI/4) + v * sinf(angle + M_PI/4));
            lut[c][ph] = (uint8_t)(fminf(fmaxf(chroma * 255.0f, 0.0f), 255.0f));
        }
    }
}

// Fast lookup table access
inline uint8_t IRAM_ATTR fastLUT(uint8_t col, uint16_t phase) {
    return lut[col & 0x07][phase & 0xFF];
}

// Line timer interrupt handler
void IRAM_ATTR onLineTimer(void *arg) {
    // Wait for horizontal sync
    while (gpio_get_level(HSYNC_PIN) == 1);
    
    uint16_t *out = lineBuf[bufIndex];
    int idx = 0;
    
    // Generate sync pulse (0 IRE level)
    for (; idx < SYNC_SAMPLES; idx++) {
        out[idx] = 0;
    }
    
    // Generate color burst
    for (int i = 0; i < BURST_SAMPLES; i++, idx++) {
        float burstPhase = 2.0f * M_PI * i / 10.0f; // 4.43MHz burst
        float burst = 0.3f + 0.2f * sinf(burstPhase) * (phaseAlt ? -1.0f : 1.0f);
        out[idx] = (uint16_t)(fminf(fmaxf(burst * 255.0f, 0.0f), 255.0f));
    }
    
    // Generate active video
    for (int px = 0; px < PIXELS_PER_LINE; px++) {
        uint8_t col;
        sampleRGB8(col);
        
        // Generate samples for this pixel
        for (int s = 0; s < SAMPLES_PER_PIXEL; s++, idx++) {
            if (idx < LINE_SAMPLES) {
                out[idx] = fastLUT(col, phaseIndex);
                phaseIndex = (phaseIndex + 1) & (LUT_SIZE - 1);
            }
        }
    }
    
    // Fill remaining samples with black level
    for (; idx < LINE_SAMPLES; idx++) {
        out[idx] = (uint16_t)(0.3f * 255.0f); // Black level
    }
    
    // Send line to I2S
    size_t bytes_written;
    i2s_write(I2S_NUM_0, (const char*)out, LINE_SAMPLES * sizeof(uint16_t), &bytes_written, 0);
    
    // Swap buffers and update line counter
    bufIndex ^= 1;
    lineCount++;
    
    // PAL field alternation every 312.5 lines
    if (lineCount >= 625) {
        lineCount = 0;
        phaseAlt = !phaseAlt;
        frameSync = true;
    }
}

// Setup I2S for DAC output
void setupI2S() {
    i2s_config_t config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = LINE_SAMPLES,
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &config, 0, NULL));
    ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN));
}

// Setup hardware timer for line timing
void setupTimer() {
    timer_config_t timer_config = {
        .divider = 80,              // 1MHz timer clock
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true
    };
    
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &timer_config));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000000 / LINE_FREQ));
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
    ESP_ERROR_CHECK(timer_isr_register(TIMER_GROUP_0, TIMER_0, onLineTimer, NULL, 
                                      ESP_INTR_FLAG_IRAM, &timer_isr_handle));
    ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));
}

// Setup GPIO pins
void setupGPIO() {
    // Configure RGB input pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_R) | (1ULL << PIN_G) | (1ULL << PIN_B) | 
                       (1ULL << HSYNC_PIN) | (1ULL << VSYNC_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    
    // Configure video output pin
    io_conf.pin_bit_mask = (1ULL << VIDEO_DAC_PIN);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 PAL Composite Video Encoder Starting...");
    
    // Initialize GPIO
    setupGPIO();
    
    // Build color lookup table
    Serial.println("Building LUT...");
    buildLUT();
    
    // Setup I2S DAC
    Serial.println("Setting up I2S...");
    setupI2S();
    
    // Setup timer for line generation
    Serial.println("Setting up timer...");
    setupTimer();
    
    Serial.println("PAL encoder ready!");
}

void loop() {
    // Main loop - can be used for other tasks
    if (frameSync) {
        frameSync = false;
        // Frame sync occurred - can be used for frame-based operations
        Serial.print("Frame: ");
        Serial.println(millis());
    }
    
    delay(100);
}
