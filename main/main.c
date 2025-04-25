#include <assert.h>
#include <stdlib.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

const unsigned SOUND_BUF_SZ = 512;
typedef unsigned short uint16_t;

void read_mic_samples(float *const buffer, adc1_channel_t channel,
                      unsigned buf_sz, uint32_t t_quantum_ms) {
  assert(buffer && "Buffer must be not NULL");

  for (uint16_t i = 0; i < buf_sz; i++) {
    buffer[i] = (float) adc1_get_raw(channel);

    vTaskDelay(pdMS_TO_TICKS(t_quantum_ms));
  }
}

float calc_mean(const float * const arr, unsigned size) {
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

  float *input = malloc(SOUND_BUF_SZ * sizeof(float));
  float *spectre = malloc(SOUND_BUF_SZ * sizeof(float));

  while(1) {
    gpio_set_level(GPIO_NUM_26, 0);
    read_mic_samples(input, ADC1_CHANNEL_6, SOUND_BUF_SZ, 1);

    // normalization (0 .. 4095 -> -1.0 .. +1.0)
    for (uint16_t i = 0; i < SOUND_BUF_SZ; i++) {
      input[i] = ((float) input[i] - 2048.0f) / 2048.0f;
    }

    // calc mean
    float mean = calc_mean(input, SOUND_BUF_SZ);

    if (mean > 0) gpio_set_level(GPIO_NUM_26, 1);

    // apply fft

    // ...

    // output
    printf("mean = %.2f\n", mean);
    printf("\n");
    printf("\n");
    printf("\n");

    vTaskDelay(pdMS_TO_TICKS(100));
  }

  gpio_set_level(GPIO_NUM_26, 0);

  free(input);
  free(spectre);
}