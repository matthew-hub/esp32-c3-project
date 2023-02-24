// ESP-IDF:
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <driver/gptimer.h>
#include <driver/gpio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
// COMPONENT:
#include <button.h>
#include <aht.h>
// HEADER:
#include "num_pattern.h"

static const char *TAG = "74HC595N";

// 74HC595N PIN DESCRIPTION | SHIFT REGISTER
#define SER_PIN GPIO_NUM_20   // SERIAL DATA INPUT PIN
#define SH_CP_PIN GPIO_NUM_21 // CLOCK INPUT PIN
#define ST_CP_PIN GPIO_NUM_2  // LATCH PIN
// DISPLAY DIGIT GPIOs
#define DIG_1_PIN GPIO_NUM_1
#define DIG_2_PIN GPIO_NUM_5
#define DIG_3_PIN GPIO_NUM_4
#define DIG_4_PIN GPIO_NUM_6

// LSB (least significant bit) first or MSB (most significant bit) first
#define LSBFIRST 0
#define MSBFIRST 1

// aht i2c address "0x38" GND
#define AHT_ADDR AHT_I2C_ADDRESS_GND
// aht type "aht20"
#define AHT_TYPE AHT_TYPE_AHT20
// SDA SCL aht pin
#define AHT_SDA_PIN GPIO_NUM_8
#define AHT_SCL_PIN GPIO_NUM_9

// array of digits pin
const uint8_t DIG_PINS[4] = {DIG_1_PIN, DIG_2_PIN, DIG_3_PIN, DIG_4_PIN};
// array of numbers to display
uint8_t NUMBERS_TO_DISPLAY[4] = {2, 4, 8, 3};
// order of the digit
int ORDER_DIGITS = 0;

static const char *states[] = {
    [BUTTON_PRESSED] = "pressed",
    [BUTTON_RELEASED] = "released",
    [BUTTON_CLICKED] = "clicked",
    [BUTTON_PRESSED_LONG] = "pressed long",
};

static button_t btn1;

typedef struct
{
  uint64_t event_count;
} example_queue_element_t;

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

// display number with multiplexing 4 digits
static bool IRAM_ATTR multiplexing_display_data(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
  BaseType_t high_task_awoken = pdFALSE;

  // turn off the previous digit
  if (ORDER_DIGITS == 0) // if 0 turn the last one
    gpio_set_level(DIG_PINS[3], 1);
  else
    gpio_set_level(DIG_PINS[ORDER_DIGITS - 1], 1);

  // set pin low to send register | ST_CP_PIN = latch
  gpio_set_level(ST_CP_PIN, 0);
  // send byte of data | SER_PIN = DATA, SH_CP_PINT = CLOCK
  shift_out(SER_PIN, SH_CP_PIN, MSBFIRST, NUMBERS_PATTERN[NUMBERS_TO_DISPLAY[ORDER_DIGITS]]);
  // set pin high to save storage register
  gpio_set_level(ST_CP_PIN, 1);

  // turn on the correct digit
  gpio_set_level(DIG_PINS[ORDER_DIGITS], 0);

  ORDER_DIGITS++;
  if (ORDER_DIGITS == 4)
    ORDER_DIGITS = 0;

  // return whether we need to yield at the end of ISR
  return (high_task_awoken == pdTRUE);
}

static void on_button(button_t *btn, button_state_t state)
{
  ESP_LOGI(TAG, "%s button %s", btn == &btn1 ? "First" : "Second", states[state]);
}

void aht20_task(void *pvParameters)
{
  aht_t dev = {0};
  dev.mode = AHT_MODE_NORMAL;
  dev.type = AHT_TYPE;

  ESP_ERROR_CHECK(aht_init_desc(&dev, AHT_ADDR, 0, AHT_SDA_PIN, AHT_SCL_PIN));
  ESP_ERROR_CHECK(aht_init(&dev));

  bool calibrated;
  ESP_ERROR_CHECK(aht_get_status(&dev, NULL, &calibrated));
  if (calibrated)
    ESP_LOGI(TAG, "Sensor calibrated");
  else
    ESP_LOGW(TAG, "Sensor not calibrated!");

  float temperature, humidity;

  while (1)
  {
    esp_err_t res = aht_get_data(&dev, &temperature, &humidity);
    if (res == ESP_OK)
      ESP_LOGI(TAG, "Temperature: %.1f°C, Humidity: %.2f%%", temperature, humidity);
    else
      ESP_LOGE(TAG, "Error reading data: %d (%s)", res, esp_err_to_name(res));

    vTaskDelay(pdMS_TO_TICKS(2000));
  }
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
  io_conf.pin_bit_mask = (1ULL << GPIO_NUM_0) | (1ULL << DIG_1_PIN) | (1ULL << DIG_2_PIN) | (1ULL << DIG_3_PIN) | (1ULL << DIG_4_PIN);
  // disable pull-down mode
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  // configure GPIO with the given settings
  gpio_config(&io_conf);

  // enable power supply of display
  gpio_set_level(GPIO_NUM_0, 0);
  // disable all digit of display
  gpio_set_level(DIG_1_PIN, 1);
  gpio_set_level(DIG_2_PIN, 1);
  gpio_set_level(DIG_3_PIN, 1);
  gpio_set_level(DIG_4_PIN, 1);

  // First button connected between GPIO and GND
  // pressed logic level 0, no autorepeat
  btn1.gpio = GPIO_NUM_7;
  btn1.pressed_level = 0;
  btn1.internal_pull = true;
  btn1.autorepeat = false;
  btn1.callback = on_button;

  ESP_ERROR_CHECK(button_init(&btn1));

  // create xQueue
  QueueHandle_t queue = xQueueCreate(10, sizeof(example_queue_element_t));
  if (!queue)
  {
    ESP_LOGE(TAG, "Creating queue failed");
    return;
  }

  ESP_LOGI(TAG, "Create timer handle");
  gptimer_handle_t gptimer = NULL;
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

  gptimer_alarm_config_t alarm_config1 = {
      .reload_count = 0,
      .alarm_count = 3000, // period = 1s
      .flags.auto_reload_on_alarm = true,
  };

  gptimer_event_callbacks_t cbs = {
      .on_alarm = multiplexing_display_data,
  };

  ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config1));
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, queue));
  ESP_LOGI(TAG, "Enable timer");
  ESP_ERROR_CHECK(gptimer_enable(gptimer));
  ESP_ERROR_CHECK(gptimer_start(gptimer));

  ESP_ERROR_CHECK(i2cdev_init());
  xTaskCreate(aht20_task, "AHT_20:", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL);
}
