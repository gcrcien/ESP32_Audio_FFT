#include <arduinoFFT.h>
#include <TFT_eSPI.h>
#include <SPI.h>

// ==== Pantalla TFT ====
TFT_eSPI tft = TFT_eSPI();  // TFT_eSPI usa pines definidos en User_Setup.h

// ==== Parámetros FFT ====
#define NUM_SAMPLES     512
#define SAMPLING_FREQ   44000
double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];
double vPrev[NUM_SAMPLES / 2];  // FFT anterior

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, NUM_SAMPLES, SAMPLING_FREQ);

// ==== Parámetros de gráfica ====
int fft_y_bottom = 220;
int fft_height = 100;

// ==== Waterfall ====
#define WATERFALL_HEIGHT 80
#define WATERFALL_WIDTH  480  // CORREGIDO: ancho real de pantalla
uint16_t waterfallBuffer[WATERFALL_HEIGHT][WATERFALL_WIDTH];
int waterfallIndex = 0;

int waterfall_top = fft_y_bottom + 1;

// ==== ADC ====
#define ADC_PIN 35  // ADC1_CH8

// ==== DAC ====
#define DAC_PIN 25  // DAC1: GPIO25

// ==== Ganancia ====
float fftGain_dB = 0.0;
float fftGain = pow(10.0, fftGain_dB / 20.0);

// ==== Opciones ====
bool useNoiseSubtraction = false;

// ==== Colores ====
const uint16_t colorTop  = TFT_GREEN;
const uint16_t colorRest = TFT_DARKGREEN;

// ==== Mínimo ====
int min_int(int a, int b) {
  return (a < b) ? a : b;
}

// ==== Suavizado FFT ====
float smoothMagnitude(int i, double *magnitudes, int count) {
  if (i == 0)
    return (magnitudes[i] + magnitudes[i + 1]) / 2;
  else if (i == count - 1)
    return (magnitudes[i - 1] + magnitudes[i]) / 2;
  else
    return (magnitudes[i - 1] + magnitudes[i] + magnitudes[i + 1]) / 3;
}

// ==== Color suavizado por intensidad ====

uint16_t intensityToColor1(int intensity) {
  uint8_t r, g, b;

  if (intensity == 0) {
    r = g = b = 0;
  } else if (intensity < 50) {
    r = 0;
    g = 0;
    b = map(intensity, 0, 49, 0, 255);
  } else if (intensity < 110) {
    r = 0;
    g = map(intensity, 64, 109, 0, 255);
    b = 255;
  } else if (intensity < 180) {
    r = map(intensity, 128, 179, 0, 255);
    g = 255;
    b = map(intensity, 128, 191, 255, 0);
  } else {
    r = 255;
    g = map(intensity, 192, 255, 255, 0);
    b = 0;
  }

  return tft.color565(b, r, g);
}
uint16_t intensityToColor2(int intensity) {
  float i = constrain(intensity, 0, 255) / 255.0f;
  float r, g, b;

  // Fórmulas aproximadas para colores suaves tipo viridis (adaptado)
  r = 0.5f + 0.5f * sin(3.0f * PI * i);
  g = 0.5f + 0.5f * sin(3.0f * PI * i + 2.0f);
  b = 0.5f + 0.5f * sin(3.0f * PI * i + 4.0f);

  // Normalizar a 0-255
  uint8_t red   = uint8_t(r * 255.0f);
  uint8_t green = uint8_t(g * 255.0f);
  uint8_t blue  = uint8_t(b * 255.0f);

  return tft.color565(blue, red, green);
}

uint16_t intensityToColor3(int intensity) {
  float t = constrain(intensity, 0, 255) / 255.0f;

  // Turbo colormap aproximado (5 puntos clave interpolados)
  float r = 34 + t * (255 - 34);     // rojo crece
  float g = t < 0.5 ? (t * 2) * 255 : (1 - (t - 0.5) * 2) * 255; // verde tipo campana
  float b = 255 - t * 255;           // azul decrece

  return tft.color565(uint8_t(b), uint8_t(r), uint8_t(g));
}



// ==== Dibuja línea FFT ====
void drawFFTLine(double *magnitudes, int y_bottom, int height) {
  static int prevHeights[480];
  static double vPrevInterp[480];

  const int width = 480;
  const int bins = NUM_SAMPLES / 2;
  const float scale = (float)height / 1500.0f;
  const float bin_scale = (float)(bins - 1) / width;

  tft.startWrite();

  for (int x = 0; x < width; x++) {
    float bin_f = x * bin_scale;
    int bin_i = (int)bin_f;
    if (bin_i >= bins - 1) bin_i = bins - 2;

    float frac = bin_f - bin_i;

    float magL = smoothMagnitude(bin_i, magnitudes, bins);
    float magR = smoothMagnitude(bin_i + 1, magnitudes, bins);
    float current = magL + frac * (magR - magL);

    if (useNoiseSubtraction) {
      current -= vPrevInterp[x];
      if (current < 0) current = 0;
      vPrevInterp[x] = current;
    }

    int barHeight = (int)(current * fftGain * scale);
    if (barHeight > height) barHeight = height;

    int prevHeight = prevHeights[x];

    int y1 = y_bottom - barHeight;
    int y2 = y_bottom - prevHeight;

    if (barHeight < prevHeight) {
      tft.drawLine(x, y2, x, y1, TFT_BLACK);
    } else if (barHeight > prevHeight) {
      if (barHeight > 1) {
        tft.drawLine(x, y1 + 1, x, y2, colorRest);
      }
      tft.drawPixel(x, y1, colorTop);
    }

    prevHeights[x] = barHeight;
  }

  tft.endWrite();
}

//==== Dibujar waterfall ====

void drawWaterfallLine(double* magnitudes) {
  static uint16_t interpLine[WATERFALL_WIDTH];

  // Interpolar magnitudes a línea de 480 píxeles
  for (int x = 0; x < WATERFALL_WIDTH; x++) {
    float bin_f = ((float)x / (WATERFALL_WIDTH - 1)) * ((NUM_SAMPLES / 2) - 1);
    int bin_i = (int)bin_f;
    if (bin_i >= (NUM_SAMPLES / 2 - 1)) bin_i = (NUM_SAMPLES / 2 - 2);

    float frac = bin_f - bin_i;

    float magL = smoothMagnitude(bin_i, magnitudes, NUM_SAMPLES / 2);
    float magR = smoothMagnitude(bin_i + 1, magnitudes, NUM_SAMPLES / 2);
    float current = magL + frac * (magR - magL);
    current *= fftGain;
    int intensity = constrain((current / 800.0f) * 255, 0, 255);
    interpLine[x] = intensityToColor1(intensity);
  }

  // Guardar línea nueva en buffer circular
  memcpy(waterfallBuffer[waterfallIndex], interpLine, sizeof(interpLine));
  tft.startWrite();

  // Dibujar de arriba hacia abajo (más reciente arriba)
  for (int row = 0; row < WATERFALL_HEIGHT; row++) {
    // Calculamos índice en buffer circular: línea más nueva es waterfallIndex
    int srcIndex = (waterfallIndex - row + WATERFALL_HEIGHT) % WATERFALL_HEIGHT;
    int dstY = waterfall_top + row;

    if (dstY < tft.height()) {
      tft.pushImage(0, dstY, WATERFALL_WIDTH, 1, waterfallBuffer[srcIndex]);
    }
  }
  tft.endWrite();

  // Avanzar índice circular
  waterfallIndex = (waterfallIndex + 1) % WATERFALL_HEIGHT;
}


// ==== Captura ADC ====
void captureSamples() {
  unsigned long samplingPeriod_us = 1000000.0 / SAMPLING_FREQ;
  unsigned long t_start = micros();

  for (int i = 0; i < NUM_SAMPLES; i++) {
    vReal[i] = analogRead(ADC_PIN);
    vImag[i] = 0.0;

    while (micros() - t_start < samplingPeriod_us * (i + 1)) {
      // Esperar
    }
  }

  // Quitar offset DC
  double mean = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    mean += vReal[i];
  }
  mean /= NUM_SAMPLES;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    vReal[i] -= mean;
  }
}

// ==== Setup ====
void setup() {
  analogReadResolution(12);

  memset(vPrev, 0, sizeof(vPrev));
  memset(waterfallBuffer, 0, sizeof(waterfallBuffer));

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("ESP32 FFT ADC");
}

// ==== Loop ====
void loop() {
  captureSamples();

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, NUM_SAMPLES);

  drawFFTLine(vReal, fft_y_bottom, fft_height);
  drawWaterfallLine(vReal);
}
