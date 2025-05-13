#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "kiss_fftr.h"

const unsigned SOUND_BUF_SZ = 128;
typedef unsigned short uint16_t;

const float LIGHT_ON_THRESHOLD = 180000.0f;

struct light_controller {
  float val;
  int light_on; // false by default
};

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

static void *control_light(void * light_info) {
  assert(light_info);

  int *light_on = &(((struct light_controller *) light_info)->light_on);

  while(1) {
    float level = ((struct light_controller *) light_info)->val;
    if (level >= LIGHT_ON_THRESHOLD) {
      *(int *) light_on = 1;
    } else {
      *(int *) light_on = 0;
    }

    printf ("control light : level = %.2f desicion = %d\n", level, *light_on);

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void app_main() {
  gpio_reset_pin(GPIO_NUM_26);
  gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);

  gpio_reset_pin(GPIO_NUM_14);
  gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  float *input = calloc(SOUND_BUF_SZ, sizeof(float));
  float spectre[SOUND_BUF_SZ / 2 + 1] = {};

  kiss_fftr_cfg cfg = kiss_fftr_alloc(SOUND_BUF_SZ, 0, NULL, NULL);
  kiss_fft_cpx *cx_out = (kiss_fft_cpx *) calloc(SOUND_BUF_SZ / 2 + 1, sizeof(kiss_fft_cpx));

  struct light_controller light_info;
  light_info.val = 0.0f;
  light_info.light_on = 0;

  pthread_t light_controller_thr;
  pthread_create(&light_controller_thr, NULL, control_light, (void *) &light_info);

  while(1) {
    gpio_set_level(GPIO_NUM_26, 0);
    gpio_set_level(GPIO_NUM_14, 0);
    read_mic_samples(input, ADC1_CHANNEL_6, SOUND_BUF_SZ, 1);

    light_info.val = 0;

    // normalization (0 .. 4095 -> -1.0 .. +1.0)
    for (uint16_t i = 0; i < SOUND_BUF_SZ; i++) {
      // turn on light based on volume
      light_info.val += input[i];
      // normalize
      input[i] = ((float) input[i] - 2048.0f) / 2048.0f;
    }
    // apply fft
    kiss_fftr(cfg, input, cx_out);

    for (int i = 0; i < SOUND_BUF_SZ / 2 + 1; i++) {
      float real = cx_out[i].r;
      float imag = cx_out[i].i;
      spectre[i] = sqrt(real * real + imag * imag);
    }

    float bass_level = 0.0f;
    int min_bin = 0;
    int max_bin = 30;
    for (int i = min_bin; i <= max_bin; i++) {
      bass_level += spectre[i];
    }

    gpio_set_level(GPIO_NUM_14, light_info.light_on);

    if (bass_level > 95) {
      gpio_set_level(GPIO_NUM_26, 1);
      vTaskDelay(pdMS_TO_TICKS(50));
    }

    // output
    printf("bass = %.2f | mean = %.2f\n", bass_level, light_info.val);

    vTaskDelay(pdMS_TO_TICKS(30));
  }

  gpio_set_level(GPIO_NUM_26, 0);

  free(input);
  free(cfg);
  free(cx_out);
  free(spectre);
}