#include <arduinoFFT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "SPI.h"
// ==== Pantalla TFT ====
#define TFT_CS    15
#define TFT_DC    2
#define TFT_RST   4
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ==== Parámetros FFT ====
#define NUM_SAMPLES     512
#define SAMPLING_FREQ   44000
double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];
double vPrev[NUM_SAMPLES / 2];  // FFT anterior

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, NUM_SAMPLES, SAMPLING_FREQ);

#define WATERFALL_HEIGHT 119
#define WATERFALL_WIDTH  NUM_SAMPLES/2  // igual a tft.width()

uint16_t waterfallBuffer[WATERFALL_HEIGHT][WATERFALL_WIDTH];
int waterfallIndex = 0;

int waterfall_top = 121; // Y inicial donde empieza el waterfall

// ==== ADC ====
#define ADC_PIN 35  // ADC1_CH8

// ==== DAC ====
#define DAC_PIN 25  // DAC1: GPIO25

// ==== Control de ganancia ====
float fftGain_dB = 0.0;  // Entre -10 y +10 dB
float fftGain = pow(10.0, fftGain_dB / 20.0);  // Lineal

// ==== Opciones de visualización ====
bool useNoiseSubtraction = false;  // Activar o desactivar filtro de ruido

// ==== Parámetros para posición de FFT ====
int fft_y_bottom = 120;  // Posición vertical del fondo de la FFT
int fft_height = 80;    // Altura máxima de la gráfica

// colores de la FFT
const uint16_t colorTop  = ILI9341_GREEN;              // Verde intenso (borde superior)
const uint16_t colorRest = tft.color565(0, 64, 0);     // Verde oscuro (resto de la barra)
// ==== Función para evitar errores con min() ====
int min_int(int a, int b) {
  return (a < b) ? a : b;
}

// ==== Dibuja una línea del espectro ====
// ==== Dibuja una línea del espectro ====
void drawFFTLine(double *magnitudes, int y_bottom, int height) {
  int bins_to_draw = min_int(NUM_SAMPLES / 2, tft.width());
  static int prevHeights[NUM_SAMPLES / 2];
  static double vPrev[NUM_SAMPLES / 2];

  float scale = (float)height / 1500.0f;

  for (int i = 0; i < bins_to_draw; i++) {
    //float current = magnitudes[i];
    float current = smoothMagnitude(i, magnitudes, bins_to_draw);

    if (useNoiseSubtraction) {
      current -= vPrev[i];
      if (current < 0) current = 0;
      vPrev[i] = magnitudes[i];
    }

    current *= fftGain;
    int barHeight = current * scale;
    if (barHeight > height) barHeight = height;

    int prevHeight = prevHeights[i];

    // Borrar si disminuyó
    if (barHeight < prevHeight) {
      tft.drawLine(i, y_bottom - prevHeight, i, y_bottom - barHeight, ILI9341_BLACK);
    }

    // Dibujar nueva barra si creció
    else if (barHeight > prevHeight) {
      if (barHeight > 1) {
        // Dibujar parte inferior con colorRest
        tft.drawLine(i, y_bottom - barHeight + 1, i, y_bottom - prevHeight, colorRest);
      }
      // Dibujar la línea superior (el borde más alto) con colorTop
      tft.drawPixel(i, y_bottom - barHeight, colorTop);
    }

    prevHeights[i] = barHeight;
  }
}


////=== intensity gradient====
uint16_t intensityToColor(int intensity) {
  uint8_t r, g, b;

  if (intensity == 0) {
    // Ausencia de señal (negro)
    r = 0;
    g = 0;
    b = 0;
  } else if (intensity < 64) {
    // Azul oscuro a azul
    r = 0;
    g = 0;
    b = map(intensity, 0, 63, 0, 255);
  } else if (intensity < 128) {
    // Azul a cian
    r = 0;
    g = map(intensity, 64, 127, 0, 255);
    b = 255;
  } else if (intensity < 192) {
    // Cian a amarillo
    r = map(intensity, 128, 191, 0, 255);
    g = 255;
    b = map(intensity, 128, 191, 255, 0);
  } else {
    // Amarillo a rojo
    r = 255;
    g = map(intensity, 192, 255, 255, 0);
    b = 0;
  }

  return tft.color565(r, g, b);
}
//==== Suavizado de fft====

float smoothMagnitude(int i, double *magnitudes, int count) {
  if (i == 0)
    return (magnitudes[i] + magnitudes[i + 1]) / 2;
  else if (i == count - 1)
    return (magnitudes[i - 1] + magnitudes[i]) / 2;
  else
    return (magnitudes[i - 1] + magnitudes[i] + magnitudes[i + 1]) / 3;
}


//====Waterfall==
void drawWaterfallLine(double* magnitudes) {
  int bins_to_draw = min_int(NUM_SAMPLES / 2, WATERFALL_WIDTH);

  // Guardar nueva línea en el buffer circular
  for (int x = 0; x < bins_to_draw; x++) {
    double current = magnitudes[x] * fftGain;
    int colorIntensity = constrain((current / 800.0) * 255, 0, 255);
    waterfallBuffer[waterfallIndex][x] = intensityToColor(colorIntensity);
  }

  // === Desplazar contenido hacia abajo ===
  for (int row = WATERFALL_HEIGHT ; row > 0; row--) {
    int srcIndex = (waterfallIndex + row - 1) % WATERFALL_HEIGHT;
    int dstY = waterfall_top - row + fft_y_bottom;
    tft.drawRGBBitmap(0, dstY, waterfallBuffer[srcIndex], WATERFALL_WIDTH, 1);
  }

  // === Dibujar la nueva línea arriba ===
  tft.drawRGBBitmap(0, waterfall_top, waterfallBuffer[waterfallIndex], WATERFALL_WIDTH, 1);

  // Avanzar índice circular
  waterfallIndex = (waterfallIndex + 1) % WATERFALL_HEIGHT;
}




// ==== Captura de muestras desde el ADC ====
void captureSamples() {
  unsigned long samplingPeriod_us = 1000000.0 / SAMPLING_FREQ;
  unsigned long t_start = micros();

  for (int i = 0; i < NUM_SAMPLES; i++) {
    vReal[i] = analogRead(ADC_PIN);
    vImag[i] = 0.0;

    while (micros() - t_start < samplingPeriod_us * (i + 1)) {
      // Esperar al siguiente periodo de muestreo
    }
  }
}

// ==== Setup ====
void setup() {
  analogReadResolution(12); // Resolución máxima del ADC en ESP32

  memset(vPrev, 0, sizeof(vPrev)); // Iniciar FFT anterior en cero
  memset(waterfallBuffer, 0, sizeof(waterfallBuffer));

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("ESP32 FFT ADC");
}

// ==== Loop ====
void loop() {
  captureSamples();  // Obtener muestras del ADC

  // Procesar FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, NUM_SAMPLES);

  // Opcional: anular el ruido de baja frecuencia
  vReal[0] = 0;
  vReal[1] = 0;
  vReal[2] = 0;

  // Dibujar espectro en pantalla en la posición deseada
  drawFFTLine(vReal, fft_y_bottom, fft_height);
  drawWaterfallLine(vReal);

}
