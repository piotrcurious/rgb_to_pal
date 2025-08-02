# rgb_to_pal
Simple 8 color digital RGB to PAL encoder 


📌 Wiring Recap

Signal	ESP32 Pin	Description

RGB2 (R)	GPIO32	MSB color input
RGB1 (G)	GPIO33	
RGB0 (B)	GPIO34	LSB color input
HSYNC	GPIO26	Active low scanline pulse
VSYNC	GPIO27	Active low frame pulse
Composite	GPIO25	Output via DAC1 → 100Ω → RCA tip
GND	GND	RCA shield



---

✅ Result

Composite PAL video with correct color burst

RGB input sampled in sync with external HSYNC

Full chroma support for 8 RGB colors

Accurate timing from I2S with 14.7456 MHz
