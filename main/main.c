/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_sleep.h"

/**
 * Brief:
 * This test code shows how to configure gpio and how to use gpio interrupt.
 *
 * GPIO status:
 * GPIO18: output (ESP32C2/ESP32H2 uses GPIO8 as the second output pin)
 * GPIO19: output (ESP32C2/ESP32H2 uses GPIO9 as the second output pin)
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Note. These are the default GPIO pins to be used in the example. You can
 * change IO pins in menuconfig.
 *
 * Test:
 * Connect GPIO18(8) with GPIO4
 * Connect GPIO19(9) with GPIO5
 * Generate pulses on GPIO18(8)/19(9), that triggers interrupt on GPIO4/5
 *
 */

static const char *TAG = "chickendoor";

#define STEPPER_OUTPUT_A    4
#define STEPPER_OUTPUT_B    16
#define STEPPER_OUTPUT_C    12
#define STEPPER_OUTPUT_D    14
#define HBRIDGE_ENABLE    13
#define GPIO_STEPPER_PIN_SEL ((1ULL<<STEPPER_OUTPUT_A) | (1ULL<<STEPPER_OUTPUT_B) | (1ULL<<STEPPER_OUTPUT_C) | (1ULL<<STEPPER_OUTPUT_D) | (1ULL<<HBRIDGE_ENABLE))

#define GPIO_SW_UPPER   0
#define GPIO_SW_LOWER   2
#define GPIO_SW_CTL_A     15
#define GPIO_SW_CTL_B     25
#define GPIO_LIM_SW_PIN_SEL  ((1ULL<<GPIO_SW_UPPER) | (1ULL<<GPIO_SW_LOWER))
#define GPIO_CTL_SW_PIN_SEL  ((1ULL<<GPIO_SW_CTL_A) | (1ULL<<GPIO_SW_CTL_B))



SemaphoreHandle_t xSemaphore = NULL;
esp_timer_handle_t periodic_timer;

static QueueHandle_t gpio_evt_queue = NULL;

/* Program State */
static struct machine_state {
    uint8_t upper_limit_switch;
    uint8_t lower_limit_switch; 
    enum Mode {MODE_MANUAL, MODE_AUTO, MODE_UNKNOWN} mode;
    enum Controller {CTRL_RAISE, CTRL_LOWER, CTRL_AUTO, CTRL_UNKNOWN} controller;
    enum State {STATE_OPEN, STATE_OPENING, STATE_CLOSED, STATE_CLOSING, STATE_UNKNOWN} active_state;
};

static struct machine_state state = {
    .lower_limit_switch = 0,
    .upper_limit_switch = 0,
    .mode = MODE_UNKNOWN,
    .controller = CTRL_UNKNOWN,
    .active_state = STATE_UNKNOWN,
};

void set_stepper_pins(int a, int b, int c, int d)
{
    gpio_set_level(STEPPER_OUTPUT_A, a);
    gpio_set_level(STEPPER_OUTPUT_B, b);
    gpio_set_level(STEPPER_OUTPUT_C, c);
    gpio_set_level(STEPPER_OUTPUT_D, d);
}

static void periodic_timer_callback(void* arg)
{
    static int current_state = 1;

    if(state.controller == CTRL_LOWER) {
        current_state--;
    }else if (state.controller == CTRL_RAISE) {
        current_state++;
    }
    if (current_state > 4)current_state = 1;
    if (current_state < 1)current_state = 4;

    switch(current_state)
    {
        case 1:
        set_stepper_pins(1,0,0,0);
        break;

        case 2:
        set_stepper_pins(0,0,1,0);
        break;

        case 3:
        set_stepper_pins(0,1,0,0);
        break;

        case 4:
        set_stepper_pins(0,0,0,1);
        break;

        default: break;
    }
}

/* Task function declarations */
static void task_RunMotor(void* arg);
static void ReadSwitches();
static void task_ReadClock(void* arg);
static void task_ExecuteStateMachine(void* arg);

/* All other function declarations */
static void manualStateMachine(struct machine_state* s);

void pulse_pin(uint16_t gpio, uint16_t duration_ms)
{
    gpio_set_level(gpio, 1);
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
    gpio_set_level(gpio, 0);
}

static void activate_motor()
{
    gpio_set_level(HBRIDGE_ENABLE,1);
    printf("activate motor\n");
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 6000));
}

static void deactivate_motor()
{
    gpio_set_level(HBRIDGE_ENABLE,0);
    printf("deactivate motor\n");
    set_stepper_pins(0,0,0,0);
    ESP_ERROR_CHECK(esp_timer_stop(periodic_timer));
}

static void ReadSwitches()
{
    // Check upper limit switch
    uint8_t upper_sw_status = gpio_get_level(GPIO_SW_UPPER);
    uint8_t lower_sw_status = gpio_get_level(GPIO_SW_LOWER);
    uint8_t control_sw_A = gpio_get_level(GPIO_SW_CTL_A);
    uint8_t control_sw_B = gpio_get_level(GPIO_SW_CTL_B);

    state.lower_limit_switch = lower_sw_status;
    state.upper_limit_switch = upper_sw_status;
    if(control_sw_A == 1)state.controller = CTRL_RAISE; 
    if(control_sw_B == 1)state.controller = CTRL_LOWER; 
    if((control_sw_A == 0) && (control_sw_B == 0))state.controller = CTRL_AUTO;
    // printf("CTL A: %d, CTL B: %d, L1: %d, L2: %d\n",control_sw_A,control_sw_B,lower_sw_status,upper_sw_status);
}

void print_state(struct machine_state* s)
{
    printf("state: ");
    switch(s->active_state) {
        case STATE_OPEN: printf("open ");break;
        case STATE_OPENING: printf("opening ");break;
        case STATE_CLOSED: printf("closed ");break;
        case STATE_CLOSING: printf("closing ");break;
        case STATE_UNKNOWN: printf("unkown");break;
    }
    printf("; lim_sw_lower: %d; lim_sw_upper: %d ",s->lower_limit_switch,s->upper_limit_switch);

    printf("controller: ");
    switch(s->controller){
        case CTRL_AUTO: printf("auto ");break;
        case CTRL_LOWER: printf("lower ");break;
        case CTRL_RAISE: printf("raise ");break;
        case CTRL_UNKNOWN: printf("unknown ");break;
    }
    printf("\n");
}

static void manual_lower(void)
{
    activate_motor();
    printf("Waiting for lower lim switch...\n");
    while(state.lower_limit_switch){
        ReadSwitches(NULL);
        if(state.controller != CTRL_LOWER)
        {
            printf("lower action interrupted by controller\n");
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    deactivate_motor();
}

static void manual_raise(void)
{
    activate_motor();
    printf("Waiting for upper lim switch...\n");
    while(state.upper_limit_switch){
        ReadSwitches(NULL);
        if(state.controller != CTRL_RAISE)
        {
            printf("raise action interrupted by controller\n");
            break;
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    deactivate_motor();
}

void set_wakeup_mode_any_high()
{
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(GPIO_CTL_SW_PIN_SEL,ESP_EXT1_WAKEUP_ANY_HIGH));
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    rtc_gpio_pullup_dis(GPIO_SW_CTL_A);
    rtc_gpio_pulldown_en(GPIO_SW_CTL_A);
    rtc_gpio_pullup_dis(GPIO_SW_CTL_B);
    rtc_gpio_pulldown_en(GPIO_SW_CTL_B);
}

void set_wakeup_mode_all_low()
{
    ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(GPIO_CTL_SW_PIN_SEL,ESP_EXT1_WAKEUP_ALL_LOW));
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    rtc_gpio_pullup_dis(GPIO_SW_CTL_A);
    rtc_gpio_pulldown_en(GPIO_SW_CTL_A);
    rtc_gpio_pullup_dis(GPIO_SW_CTL_B);
    rtc_gpio_pulldown_en(GPIO_SW_CTL_B);
}

typedef struct  {
    int door_open_time;
    int door_close_time;
}timetable_t;

static const timetable_t timetable = {
    .door_open_time = 07,
    .door_close_time = 20,
};

static void deep_sleep_register_rtc_timer_wakeup(int hours)
{
    int wakeup_time_sec = hours * 60 * 60;
    printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000));
}

int get_timetable_state()
{
    
}

void app_main(void)
{
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic"
    };
    
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));


    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_STEPPER_PIN_SEL;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //enable interrupt
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_CTL_SW_PIN_SEL;
    io_conf.pull_up_en = 0;
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    //enable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = GPIO_LIM_SW_PIN_SEL;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);




    while(1)
    {
        vTaskDelay(200 / portTICK_PERIOD_MS); // debounce delay
        ReadSwitches();
        print_state(&state);
        int current_time = 0;
        // If we are in manual open or manual close, wake on all low (center pos)
        switch (state.controller)
        {
            case CTRL_LOWER:
                printf("Detected state: manual lower\n");
                set_wakeup_mode_all_low();
                manual_lower();
                break;
            case CTRL_RAISE:
                printf("Detected state: manual raise\n");
                set_wakeup_mode_all_low();
                manual_raise();
                break;
            case CTRL_AUTO:
                printf("Detected state: auto\n");
                // check door state.
                    if(current_time > timetable.door_open_time &&
                        current_time < timetable.door_close_time){
                            if(state.upper_limit_switch == 0U) { // door is open
                                manual_raise();
                            }
                        }else {
                            if(state.lower_limit_switch == 0U) {
                                manual_lower();
                            }
                        }
                }
                // check the timetable
                // match to the door to the timetable
                set_wakeup_mode_any_high();
                break;
            default:
                printf("undefined controller state\n");
                break;
        }

        printf("finished door control. entering sleep..\n");
        esp_deep_sleep_start();
        esp_wake_deep_sleep();
        printf("woke from sleep\n");
    }

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    vSemaphoreCreateBinary( xSemaphore );
    if( xSemaphore == NULL )
    {
        printf("Error creating semaphore\n");
    }
    state.active_state = STATE_UNKNOWN;
}
