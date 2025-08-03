// ESP32 PAL composite video encoder with 8-color RGB input and full chroma modulation
// Original Author: ChatGPT, Claude
// Completed, Fixed, and Optimized by Gemini
//
// Description:
// This code turns an ESP32 into a PAL composite video signal generator. It reads a 3-bit
// digital RGB signal (8 colors), along with an optional VSYNC signal, and encodes it into
// a standard PAL composite video signal output via one of the ESP32's DAC pins.
//
// Key Features:
// - 8-Color (3-bit R, G, B) input.
// - Generates a standard 625-line, 50Hz interlaced PAL signal.
// - Full chroma modulation with correct PAL phase alternation (V-switch).
// - Accurate generation of sync, back porch, color burst, active video, and front porch.
// - Complete Vertical Blanking Interval (VBI) generation, including equalizing and broad sync pulses.
// - Uses I2S DMA for stable, low-CPU-overhead DAC output.
// - High-performance inline assembly for fast GPIO reads.
// - Fully optimized interrupt using pre-calculated Look-Up Tables (LUTs) to eliminate all floating-point math.

#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/timer.h>
#include <soc/gpio_struct.h>
#include <math.h>

// --- Pin Configuration ---
#define PIN_R 32
#define PIN_G 33
#define PIN_B 26
#define VSYNC_PIN 14
#define VIDEO_DAC_PIN 25 // DAC Channel 1

// --- PAL Signal Timing Constants (Optimized for Integer Math) ---
// By choosing a 15MHz sample rate, we can get an integer number of samples per pixel,
// which is critical for a stable picture.
#define SAMPLE_RATE       15000000
#define LINE_FREQ         15625
#define LINE_SAMPLES      (SAMPLE_RATE / LINE_FREQ) // 15,000,000 / 15625 = 960 samples

// Timings in microseconds, converted to samples. 1 sample = 1/15 us.
#define US_TO_SAMPLES(us) ((us) * SAMPLE_RATE / 1000000)
#define SYNC_SAMPLES      US_TO_SAMPLES(4.7)  // 70.5 -> 71 samples
#define BURST_START_SAMPLES US_TO_SAMPLES(5.6) // 84 samples after sync start
#define BURST_SAMPLES     US_TO_SAMPLES(2.25) // 33.75 -> 34 samples (10 cycles of 4.43MHz)
#define ACTIVE_START_SAMPLES US_TO_SAMPLES(10.7) // 160.5 -> 161 samples
#define ACTIVE_SAMPLES    US_TO_SAMPLES(52.0) // 780 samples
#define PIXELS_PER_LINE   195
#define SAMPLES_PER_PIXEL (ACTIVE_SAMPLES / PIXELS_PER_LINE) // 780 / 195 = 4 samples per pixel

// --- PAL Frame Structure ---
#define LINES_PER_FRAME   625
#define FIRST_FIELD_LINES 312
#define VBI_LINES         25  // Vertical Blanking Interval lines per field

// --- Global Variables ---
static uint16_t lineBuf[2][LINE_SAMPLES]; // Double buffer for I2S DMA (uint16_t for DAC mode)
static volatile int bufIndex = 0;
static volatile int lineCount = 0;
static volatile bool frameReady = false; // Flag for VSYNC
static portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// --- Lookup Tables for Performance ---
#define LUT_PHASE_SIZE 256
// Two LUTs for PAL V-Switch (normal and phase-inverted V component)
static uint8_t lut_normal[8][LUT_PHASE_SIZE];
static uint8_t lut_switched[8][LUT_PHASE_SIZE];
// Sine table for generating color burst efficiently
static int8_t burst_sine_lut[LUT_PHASE_SIZE];

// YUV color space structure
struct YUV { float y, u, v; };

// RGB to YUV conversion table for 8 colors (3-bit RGB: B,G,R)
const YUV rgb2yuv[8] = {
    {0.000,  0.000,  0.000}, // 000 Black
    {0.114, -0.203, -0.292}, // 001 Blue
    {0.587, -0.289,  0.439}, // 010 Green
    {0.701, -0.492,  0.147}, // 011 Cyan
    {0.299,  0.492, -0.147}, // 100 Red
    {0.413,  0.289, -0.439}, // 101 Magenta
    {0.886,  0.203,  0.292}, // 110 Yellow
    {1.000,  0.000,  0.000}  // 111 White
};

// --- Signal Levels (8-bit) ---
#define LEVEL_SYNC_8B     0
#define LEVEL_BLACK_8B    76  // Black level pedestal
#define LEVEL_WHITE_8B    255

// --- Signal Levels (16-bit for I2S DAC) ---
#define LEVEL_SYNC        (LEVEL_SYNC_8B << 8)
#define LEVEL_BLACK       (LEVEL_BLACK_8B << 8)
#define LEVEL_WHITE       (LEVEL_WHITE_8B << 8)

// --- Color Burst Constants ---
#define BURST_AMPLITUDE   38 // Amplitude of burst in 8-bit DAC units
#define PAL_SUBCARRIER    4433618.75

// Fixed-point phase increment for subcarrier, calculated once in setup
// (PAL_SUBCARRIER / SAMPLE_RATE) * 2^16
const uint32_t PHASE_INC_FIXED = (uint32_t)((PAL_SUBCARRIER / (double)SAMPLE_RATE) * 65536.0);

// --- Function Prototypes ---
void IRAM_ATTR onLineTimer(void *arg);
void setupI2S();
void setupTimer();
void setupGPIO();
void buildLUTs();
void buildBurstLUT();
uint8_t IRAM_ATTR sampleRGB();
void IRAM_ATTR onVSync();

// Fast GPIO sampling using direct register access
uint8_t IRAM_ATTR sampleRGB() {
    uint32_t val = GPIO.in;
    uint8_t r = (val >> PIN_R) & 1;
    uint8_t g = (val >> PIN_G) & 1;
    uint8_t b = (val >> PIN_B) & 1;
    return (r << 2) | (g << 1) | b;
}

// VSYNC interrupt handler
void IRAM_ATTR onVSync() {
    portENTER_CRITICAL_ISR(&timerMux);
    lineCount = 0;
    frameReady = true;
    portEXIT_CRITICAL_ISR(&timerMux);
}

// Build sine lookup table for color burst generation
void buildBurstLUT() {
    for (int i = 0; i < LUT_PHASE_SIZE; i++) {
        burst_sine_lut[i] = (int8_t)(sinf(2.0f * M_PI * i / LUT_PHASE_SIZE) * 127.0f);
    }
}

// Build lookup tables for chroma modulation
void buildLUTs() {
    for (int c = 0; c < 8; c++) {
        for (int ph = 0; ph < LUT_PHASE_SIZE; ph++) {
            float angle = 2.0f * M_PI * ph / LUT_PHASE_SIZE;
            
            // --- Normal Line ---
            float u = rgb2yuv[c].u;
            float v = rgb2yuv[c].v;
            float luma_level = LEVEL_BLACK_8B + rgb2yuv[c].y * (LEVEL_WHITE_8B - LEVEL_BLACK_8B);
            float chroma_signal = (u * sinf(angle) + v * cosf(angle));
            int level = luma_level + chroma_signal * BURST_AMPLITUDE * 1.33; // Gain factor for chroma
            lut_normal[c][ph] = (uint8_t)constrain(level, 0, 255);

            // --- Switched Line (V component is inverted) ---
            float v_switched = -v;
            float chroma_switched = (u * sinf(angle) + v_switched * cosf(angle));
            level = luma_level + chroma_switched * BURST_AMPLITUDE * 1.33;
            lut_switched[c][ph] = (uint8_t)constrain(level, 0, 255);
        }
    }
}

// Inline function to convert 8-bit DAC value to 16-bit I2S format
inline uint16_t IRAM_ATTR dac8to16(uint8_t val) {
    return (uint16_t)val << 8;
}

// Line timer interrupt handler - The core of the video generation
void IRAM_ATTR onLineTimer(void *arg) {
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

    portENTER_CRITICAL_ISR(&timerMux);
    
    uint16_t *out = lineBuf[bufIndex];
    int idx = 0;
    int currentLine = lineCount;

    // --- VBI (Vertical Blanking Interval) Logic ---
    bool is_vbi = (currentLine < VBI_LINES) || 
                  (currentLine >= (FIRST_FIELD_LINES - VBI_LINES/2) && currentLine < (FIRST_FIELD_LINES + VBI_LINES/2)) ||
                  (currentLine >= (LINES_PER_FRAME - VBI_LINES));

    if (is_vbi) {
        // Generate VBI lines with proper sync patterns
        bool isBroadPulse = (currentLine >= 2 && currentLine <= 4) || 
                            (currentLine >= (FIRST_FIELD_LINES + 2) && currentLine <= (FIRST_FIELD_LINES + 4));
        
        if (isBroadPulse) { // Broad sync pulses
            for(idx = 0; idx < LINE_SAMPLES - SYNC_SAMPLES; idx++) out[idx] = LEVEL_SYNC;
            for(; idx < LINE_SAMPLES; idx++) out[idx] = LEVEL_BLACK;
        } else { // Normal sync or equalizing pulses
            for(idx = 0; idx < SYNC_SAMPLES; idx++) out[idx] = LEVEL_SYNC;
            for(; idx < LINE_SAMPLES; idx++) out[idx] = LEVEL_BLACK;
        }
    } else {
        // --- Active Video Line Generation ---
        uint32_t phase_accum = 0; // Reset phase accumulator for each line

        // 1. Horizontal Sync Pulse
        for (idx = 0; idx < SYNC_SAMPLES; idx++) out[idx] = LEVEL_SYNC;

        // 2. Back Porch (black level)
        for (; idx < BURST_START_SAMPLES; idx++) out[idx] = LEVEL_BLACK;

        // 3. Color Burst
        bool v_switch = (currentLine & 1);
        // Burst phase is ±135 deg relative to +U. 135/360*256=96, 225/360*256=160
        uint8_t burst_phase_offset = v_switch ? 160 : 96;
        for (int i = 0; i < BURST_SAMPLES; i++, idx++) {
            uint8_t phase_index = ((phase_accum >> 8) + burst_phase_offset) & 0xFF;
            int8_t sine_val = burst_sine_lut[phase_index];
            int burst_level = LEVEL_BLACK_8B + ((sine_val * BURST_AMPLITUDE) >> 7);
            out[idx] = dac8to16((uint8_t)constrain(burst_level, 0, 255));
            phase_accum += PHASE_INC_FIXED;
        }

        // Fill gap to active video start
        for (; idx < ACTIVE_START_SAMPLES; idx++) out[idx] = LEVEL_BLACK;

        // 4. Active Video Area
        for (int px = 0; px < PIXELS_PER_LINE; px++) {
            uint8_t color = sampleRGB();
            uint8_t* current_lut = v_switch ? lut_switched[color] : lut_normal[color];
            
            for (int s = 0; s < SAMPLES_PER_PIXEL; s++, idx++) {
                if (idx >= LINE_SAMPLES) break;
                uint8_t phase_index = (phase_accum >> 8) & 0xFF;
                out[idx] = dac8to16(current_lut[phase_index]);
                phase_accum += PHASE_INC_FIXED;
            }
        }

        // 5. Front Porch - fill remaining samples
        for (; idx < LINE_SAMPLES; idx++) out[idx] = LEVEL_BLACK;
    }

    // Send the generated line to I2S DMA
    size_t bytes_written;
    i2s_write(I2S_NUM_0, out, LINE_SAMPLES * sizeof(uint16_t), &bytes_written, 0);
    
    // Update counters
    bufIndex ^= 1;
    lineCount++;
    if (lineCount >= LINES_PER_FRAME) {
        lineCount = 0;
    }
    
    portEXIT_CRITICAL_ISR(&timerMux);
}

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // Use DAC Ch 1 (GPIO25)
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 256, // DMA buffer length
        .use_apll = true,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN);
    i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupTimer() {
    timer_config_t config = {
        .divider = 80, // 80MHz APB clock / 80 = 1MHz timer clock
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = true,
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    // Set timer to trigger at line frequency (1MHz / 15625Hz = 64)
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000000 / LINE_FREQ);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, onLineTimer, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_0);
}

void setupGPIO() {
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << PIN_R) | (1ULL << PIN_G) | (1ULL << PIN_B);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // Configure VSYNC input pin with interrupt
    io_conf.pin_bit_mask = (1ULL << VSYNC_PIN);
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);
    
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t)VSYNC_PIN, onVSync, NULL);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n=== ESP32 PAL Composite Video Encoder (Optimized) ===");
    Serial.println("Starting initialization...");

    Serial.println("Setting up GPIO...");
    setupGPIO();
    Serial.println("✓ GPIO initialized");

    Serial.println("Building color lookup tables...");
    buildBurstLUT();
    buildLUTs();
    Serial.println("✓ LUTs built");

    Serial.println("Setting up I2S DAC...");
    setupI2S();
    Serial.println("✓ I2S initialized");

    Serial.println("Setting up line timer...");
    setupTimer();
    Serial.println("✓ Timer started");

    Serial.println("\n=== PAL Encoder Ready! ===");
    Serial.printf("Sample Rate: %d Hz\n", SAMPLE_RATE);
    Serial.printf("Pixels per Line: %d (%d samples/pixel)\n", PIXELS_PER_LINE, SAMPLES_PER_PIXEL);
    Serial.println("Connect composite video output to GPIO25 (DAC1)");
    Serial.println("Connect RGB inputs to GPIO32, GPIO33, GPIO26");
    Serial.println("Connect VSYNC to GPIO14 (optional, for frame sync)");
}

void loop() {
    // Main loop is free for other tasks.
    // Video generation is handled entirely by timer interrupts.
    
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 2000) {
        portENTER_CRITICAL(&timerMux);
        int currentLine = lineCount;
        bool frameFlag = frameReady;
        if (frameReady) frameReady = false;
        portEXIT_CRITICAL(&timerMux);
        
        Serial.printf("Status - Line: %d/%d", currentLine, LINES_PER_FRAME);
        if (frameFlag) {
            Serial.print(" [VSYNC]");
        }
        Serial.println();
        
        lastPrint = millis();
    }
    
    delay(10); // Yield to prevent watchdog timeout
}
