📊 ESP32 Real-Time FFT + Waterfall Visualizer
This project implements a real-time frequency spectrum analyzer using FFT (Fast Fourier Transform) and a waterfall display on an ESP32 with an ILI9341 TFT screen. It captures audio or analog signals through the internal ADC, performs spectral analysis, and visualizes the result graphically.

🔧 Requirements
ESP32

ILI9341 TFT display

Arduino libraries:

arduinoFFT

Adafruit_ILI9341

Adafruit_GFX

📐 Specifications
Sampling rate: 512 samples at 44 kHz

FFT resolution: 512-point FFT (covers up to 22 kHz)

Display outputs:

Real-time FFT bar graph

Waterfall (scrolling color-based frequency map)

Color mapping: blue → cyan → yellow → red, based on signal intensity

Adjustable gain: via fftGain_dB

Optional noise subtraction (simple spectral background filtering)

ADC sampling via analogRead(), synchronized with micros() timer

📌 Default Pinout
Signal	ESP32 Pin	Description
ADC_IN	GPIO35	Analog input (ADC1_CH8)
DAC_OUT	GPIO25	DAC output (optional use)
TFT_CS	GPIO15	Chip Select for TFT
TFT_DC	GPIO2	Data/Command for TFT
TFT_RST	GPIO4	TFT Reset

🖥️ Display Interface
FFT view: vertical green bars representing signal magnitude

Waterfall view: scrolling spectral heatmap with colors for intensity

Colors are calculated with the intensityToColor() function for a smooth gradient.
![fft_1](https://github.com/user-attachments/assets/551175d5-2cd6-4819-8896-104f5c64168a)

🔍 Key Functions
captureSamples() — Samples analog data at precise intervals

drawFFTLine() — Draws vertical spectrum bars on screen

drawWaterfallLine() — Draws and scrolls new waterfall lines

intensityToColor() — Maps signal intensity to RGB color

smoothMagnitude() — Applies simple smoothing to FFT bins

🧪 Options & Features
Set useNoiseSubtraction = true to enable simple background noise filtering.

Adjust fftGain_dB to scale the FFT intensity (useful for visual tuning).

📷 Demo
(Insert a GIF or image of the waterfall + FFT display here)

📁 Related Projects
If you have other similar versions (e.g., DMA-accelerated, I2S ADC, ILI9488 support), you can reuse this structure and update:

Sampling method (analogRead() vs I2S)

Screen resolution and type

FFT size or frequency range

