#include <assert.h>
#include <stdlib.h>
#include <math.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "kiss_fftr.h"

const unsigned SOUND_BUF_SZ = 128;
typedef unsigned short uint16_t;

static void read_mic_samples(float *const buffer, adc1_channel_t channel,
                             unsigned buf_sz, uint32_t t_quantum_ms) {
  assert(buffer && "Buffer must be not NULL");

  for (uint16_t i = 0; i < buf_sz; i++) {
    buffer[i] = (float) adc1_get_raw(channel);

    vTaskDelay(pdMS_TO_TICKS(t_quantum_ms));
  }
}

static float calc_mean(const float * const arr, unsigned size) {
  assert(arr);

  float mean = 0;
  for (unsigned i = 0; i < size; i++) mean += arr[i];
  mean /= size;

  return mean;
}

void app_main() {
  gpio_reset_pin(GPIO_NUM_26);
  gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  float *input = calloc(SOUND_BUF_SZ, sizeof(float));
  float spectre[SOUND_BUF_SZ / 2 + 1] = {};

  kiss_fftr_cfg cfg = kiss_fftr_alloc(SOUND_BUF_SZ, 0, NULL, NULL);
  kiss_fft_cpx *cx_out = (kiss_fft_cpx *) calloc(SOUND_BUF_SZ / 2 + 1, sizeof(kiss_fft_cpx));

  while(1) {
    gpio_set_level(GPIO_NUM_26, 0);
    read_mic_samples(input, ADC1_CHANNEL_6, SOUND_BUF_SZ, 1);

    // normalization (0 .. 4095 -> -1.0 .. +1.0)
    for (uint16_t i = 0; i < SOUND_BUF_SZ; i++)
      input[i] = ((float) input[i] - 2048.0f) / 2048.0f;

    // apply fft
    kiss_fftr(cfg, input, cx_out);

    for (int i = 0; i < SOUND_BUF_SZ / 2 + 1; i++) {
      float real = cx_out[i].r;
      float imag = cx_out[i].i;
      spectre[i] = sqrt(real * real + imag * imag);
    }

    float bass_level = 0.0f;
    int min_bin = 0;
    int max_bin = 50;
    for (int i = min_bin; i <= max_bin; i++) {
      bass_level += spectre[i];
    }

    if (bass_level > 100) gpio_set_level(GPIO_NUM_26, 1);

    // output
    printf("bass = %.2f\n", bass_level);
    printf("\n");
    printf("\n");
    printf("\n");

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  gpio_set_level(GPIO_NUM_26, 0);

  free(input);
  free(cfg);
  free(cx_out);
  free(spectre);
}