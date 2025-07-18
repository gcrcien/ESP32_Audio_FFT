#include <arduinoFFT.h>                // Librería FFT
#include <Adafruit_GFX.h>              // Gráficos para la pantalla
#include <Adafruit_ILI9341.h>          // Controlador específico ILI9341
#include "SPI.h"

// ==== Configuración de pantalla TFT ====
#define TFT_CS    15
#define TFT_DC    2
#define TFT_RST   4
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// ==== Configuración del periférico I2S ====
#include <driver/i2s.h>
#define I2S_PORT I2S_NUM_0
#define I2S_DATA_PIN    26
#define I2S_BCK_PIN     5
#define I2S_LRCK_PIN    25

// ==== Parámetros de FFT ====
#define NUM_SAMPLES     512            // Tamaño FFT (potencia de 2)
#define SAMPLING_FREQ   150000          // Frecuencia de muestreo en Hz

// Buffers para FFT
double vReal[NUM_SAMPLES];            // Parte real
double vImag[NUM_SAMPLES];            // Parte imaginaria
static double vPrev[NUM_SAMPLES / 2]; // Buffer anterior (para substracción de ruido)
double  minVal = INT32_MAX;           // Para observar mínimo capturado
double maxVal = INT32_MIN;            // Para observar máximo capturado

// Instancia FFT con tipo double
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, NUM_SAMPLES, SAMPLING_FREQ);

// ==== Waterfall ====
#define WATERFALL_HEIGHT 119
#define WATERFALL_WIDTH  (NUM_SAMPLES/2)

uint16_t waterfallBuffer[WATERFALL_HEIGHT][WATERFALL_WIDTH];
int waterfallIndex = 0;
int waterfall_top = 121;              // Y donde inicia el gráfico de cascada

// ==== Vista de FFT ====
bool useNoiseSubtraction = false;     // Activar substracción de ruido previa

int fft_y_bottom = 120;              // Línea base del gráfico de FFT
int fft_height = 80;                 // Altura máxima del gráfico

// Colores para la gráfica FFT
const uint16_t colorTop  = ILI9341_GREEN;
const uint16_t colorRest = ILI9341_DARKGREEN;

// ==== Tabla de colores para waterfall ====
uint16_t colorLUT[256];              // LUT de colores tipo "viridis"/espectro térmico

// Genera la tabla de colores (gradiente azul → verde → rojo)
void generateColorLUT() {
  for (int intensity = 0; intensity < 256; intensity++) {
    uint8_t r, g, b;
    if (intensity < 64) {
      r = 0; g = 0; b = 4 * intensity;
    } else if (intensity < 128) {
      r = 0; g = 4 * (intensity - 64); b = 255;
    } else if (intensity < 192) {
      r = 4 * (intensity - 128); g = 255; b = 255 - 4 * (intensity - 128);
    } else {
      r = 255; g = 255 - 4 * (intensity - 192); b = 0;
    }
    colorLUT[intensity] = tft.color565(r, g, b);
  }
}

// Devuelve el menor entre dos enteros
int min_int(int a, int b) {
  return (a < b) ? a : b;
}

// Suaviza la magnitud con vecinos
float smoothMagnitude(int i, double *magnitudes, int count) {
  if (i == 0)
    return (magnitudes[i] + magnitudes[i + 1]) / 2;
  else if (i == count - 1)
    return (magnitudes[i - 1] + magnitudes[i]) / 2;
  else
    return (magnitudes[i - 1] + magnitudes[i] + magnitudes[i + 1]) / 3;
}

// === DIBUJA LA FFT ===
// Aquí se hace el ESCALADO de la magnitud FFT a píxeles
void drawFFTLine(double *magnitudes, int y_bottom, int height1) {
  int bins_to_draw = min_int(NUM_SAMPLES / 2, tft.width());

  static int prevHeights[NUM_SAMPLES / 2];

  // ESCALADO VISUAL FIJO (no autoescalado)
  float scale = (float)height1 / 3300000.0f;

  for (int i = 0; i < bins_to_draw; i++) {
    float current = smoothMagnitude(i, magnitudes, bins_to_draw);

    if (useNoiseSubtraction) {
      current -= vPrev[i];
      if (current < 0) current = 0;
      vPrev[i] = magnitudes[i];
    }

    // Escala a altura en píxeles
    int barHeight = current * scale;
    if (barHeight > height1) barHeight = height1;

    int prevHeight = prevHeights[i];

    // Dibujo en pantalla: borra o dibuja según cambio
    if (barHeight < prevHeight) {
      tft.drawLine(i, y_bottom - prevHeight, i, y_bottom - barHeight, ILI9341_BLACK);
    } else if (barHeight > prevHeight) {
      if (barHeight > 1) {
        tft.drawLine(i, y_bottom - barHeight + 1, i, y_bottom - prevHeight, colorRest);
      }
      tft.drawPixel(i, y_bottom - barHeight, colorTop);
    }

    prevHeights[i] = barHeight;
  }
}

// === DIBUJA UNA LÍNEA DE CASCADA (WATERFALL) ===
// También aplica un escalado visual
void drawWaterfallLine(double* magnitudes) {
  int bins_to_draw = min_int(NUM_SAMPLES / 2, WATERFALL_WIDTH);

  for (int x = 0; x < bins_to_draw; x++) {
    double current = magnitudes[x];

    // ESCALADO VISUAL (mapear magnitud a color)
    int intensity = constrain(map(current, 0, 3100000, 0, 255), 0, 255);
    waterfallBuffer[waterfallIndex][x] = colorLUT[intensity];
  }

  // Redibujar toda la cascada desde el buffer
  for (int row = 0; row < WATERFALL_HEIGHT; row++) {
    int srcIndex = (waterfallIndex + WATERFALL_HEIGHT - row) % WATERFALL_HEIGHT;
    int dstY = waterfall_top + row;
    tft.drawRGBBitmap(0, dstY, waterfallBuffer[srcIndex], WATERFALL_WIDTH, 1);
  }

  waterfallIndex = (waterfallIndex + 1) % WATERFALL_HEIGHT;
}

// === CONFIGURACIÓN DEL I2S ===
void setupI2S() {
  i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,    // ¡IMPORTANTE! aunque PCM1808 da 24 bits, se capturan 32
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,     // Canal derecho solamente
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll =  true,                                // Usa APLL para mejor precisión de reloj
    .tx_desc_auto_clear = true,
    .fixed_mclk = 256 * SAMPLING_FREQ                 // MCLK para ADC
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = 26,       // ¡OJO! este y el data están invertidos (probablemente error)
    .ws_io_num = 25,
    .data_out_num = -1,
    .data_in_num = 5
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
  Serial.begin(115200); 
}

// === CAPTURA DE DATOS I2S DEL ADC ===
void captureSamples() {
  const int bytes_per_sample = 4;
  const int total_bytes = NUM_SAMPLES * bytes_per_sample;
  uint8_t i2s_data[total_bytes];
  size_t bytes_read;

  i2s_read(I2S_PORT, &i2s_data, total_bytes, &bytes_read, portMAX_DELAY);

  for (int j = 0, i = 0; j < NUM_SAMPLES; j++, i += 4) {
    int32_t sample =
                   ((int32_t)(i2s_data[i + 2]) << 17) |
                   ((int32_t)(i2s_data[i + 3]) << 9);
    sample >>= 8; // Sign extension (de 24 a 32 bits con desplazamiento)

    if (sample < minVal) minVal = sample;
    if (sample > maxVal) maxVal = sample;

    vReal[j] = (double)sample;
    vImag[j] = 0.0;
  }
}

// === CONFIGURACIÓN INICIAL ===
void setup() {
  setupI2S();                    // Inicializa el ADC y reloj
  generateColorLUT();           // Crea los colores del waterfall
  memset(vPrev, 0, sizeof(vPrev));
  memset(waterfallBuffer, 0, sizeof(waterfallBuffer));

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("ESP32 FFT ADC");
}

// === LOOP PRINCIPAL ===
void loop() {
  captureSamples();                             // Obtiene datos de ADC

  FFT.windowing(FFTWindow::Blackman, FFTDirection::Forward);  // Aplica ventana Hamming
  FFT.compute(vReal, vImag, NUM_SAMPLES, FFT_FORWARD);       // Calcula FFT
  FFT.complexToMagnitude(vReal, vImag, NUM_SAMPLES);         // Convierte a magnitudes

  vReal[0] = vReal[1] = vReal[2] = 0;          // Eliminar DC y bajas frecuencias
  drawFFTLine(vReal, fft_y_bottom, fft_height);
  drawWaterfallLine(vReal);
}
