// ESP-IDF:
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
// COMPONENT:

// HEADER:
#include "num_pattern.h"

// 74HC595N PIN DESCRIPTION | SHIFT REGISTER
#define SER_PIN GPIO_NUM_20   // SERIAL DATA INPUT PIN
#define SH_CP_PIN GPIO_NUM_21 // CLOCK INPUT PIN
#define ST_CP_PIN GPIO_NUM_2  // LATCH PIN

// LSB (least significant bit) first or MSB (most significant bit) first
#define LSBFIRST 0
#define MSBFIRST 1

// config of 74HC595N pins
static esp_err_t gpio_config_shift_register()
{
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1ULL << SER_PIN) | (1ULL << SH_CP_PIN) | (1ULL << ST_CP_PIN);
  // disable pull-up mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  // disable pull-down mode
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  // configure GPIO with the given settings
  return gpio_config(&io_conf);
}

// shift_out a byte of data one bit at a time
// starts from either the most (i.e. the leftmost) or least (rightmost) significant bit
void shift_out(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t val)
{
  uint8_t i;
  // send data (bits) to 74HC595N shift register
  for (i = 0; i < 8; i++)
  {
    if (bitOrder == LSBFIRST)
    {
      gpio_set_level(dataPin, !!(val & (1 << i)));
    }
    else
    {
      gpio_set_level(dataPin, !!(val & (1 << (7 - i))));
    }
    gpio_set_level(clockPin, 1);
    gpio_set_level(clockPin, 0);
  }
}


// display number to 7 segment 4 digit display
void display_data()
{
  // set pin low to send register | ST_CP_PIN = latch
  gpio_set_level(ST_CP_PIN, 0);
  // send byte of data | SER_PIN = DATA, SH_CP_PINT = CLOCK
  shift_out(SER_PIN, SH_CP_PIN, MSBFIRST, NUMBERS_PATTERN[5]);
  // set pin high to save storage register
  gpio_set_level(ST_CP_PIN, 1);
}

void app_main(void)
{
  // set shift register gpio config
  ESP_ERROR_CHECK(gpio_config_shift_register());
  // zero-initialize the config structure.
  gpio_config_t io_conf = {};
  // disable interrupt
  io_conf.intr_type = GPIO_INTR_DISABLE;
  // set as output mode
  io_conf.mode = GPIO_MODE_OUTPUT;
  // bit mask of the pins that you want to set,e.g.GPIO18/19
  io_conf.pin_bit_mask = (1ULL << GPIO_NUM_0) | (1ULL << GPIO_NUM_1);
  // disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  // configure GPIO with the given settings
  gpio_config(&io_conf);
  
  // enable power supply of display
  gpio_set_level(GPIO_NUM_0, 0);
  // enable first digit of display
  gpio_set_level(GPIO_NUM_1, 0);
  // show data on display
  display_data();
}
