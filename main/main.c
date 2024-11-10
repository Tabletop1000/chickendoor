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
#include "freertos/semphr.h"

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

#define STEPPER_OUTPUT_A    4
#define STEPPER_OUTPUT_B    16
#define STEPPER_OUTPUT_C    12
#define STEPPER_OUTPUT_D    14
#define GPIO_STEPPER_PIN_SEL ((1ULL<<STEPPER_OUTPUT_A) | (1ULL<<STEPPER_OUTPUT_B) | (1ULL<<STEPPER_OUTPUT_C) | (1ULL<<STEPPER_OUTPUT_D))

#define GPIO_SW_UPPER   0
#define GPIO_SW_LOWER   2
#define GPIO_SW_CTL_A     15
#define GPIO_SW_CTL_B     18
#define GPIO_SW_PIN_SEL  ((1ULL<<GPIO_SW_UPPER) | (1ULL<<GPIO_SW_LOWER) | (1ULL<<GPIO_SW_CTL_A) | (1ULL<<GPIO_SW_CTL_B))

SemaphoreHandle_t xSemaphore = NULL;

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

/* Task function declarations */
static void task_RunMotor(void* arg);
static void task_ReadSwitches(void* arg);
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

void motor_forward_one_tick(uint16_t motor_delay)
{
    pulse_pin(STEPPER_OUTPUT_A, motor_delay);
    pulse_pin(STEPPER_OUTPUT_C, motor_delay);
    pulse_pin(STEPPER_OUTPUT_B, motor_delay);
    pulse_pin(STEPPER_OUTPUT_D, motor_delay);
}

void motor_reverse_one_tick(uint16_t motor_delay)
{
    pulse_pin(STEPPER_OUTPUT_D, motor_delay);
    pulse_pin(STEPPER_OUTPUT_B, motor_delay);
    pulse_pin(STEPPER_OUTPUT_C, motor_delay);
    pulse_pin(STEPPER_OUTPUT_A, motor_delay);
}

void motor_stop(void)
{
    gpio_set_level(STEPPER_OUTPUT_D, 0);
    gpio_set_level(STEPPER_OUTPUT_B, 0);
    gpio_set_level(STEPPER_OUTPUT_C, 0);
    gpio_set_level(STEPPER_OUTPUT_A, 0);
}

static void task_RunMotor(void* arg)
{
    uint16_t motor_delay = 10;
    struct machine_state local_state;
    for(;;){
        if(xSemaphoreTake(xSemaphore,portMAX_DELAY)){
            local_state = state;
            xSemaphoreGive(xSemaphore);
        }else{
            continue;
        }
        switch(state.active_state)
        {
            case STATE_OPENING:
                motor_forward_one_tick(motor_delay);
                break;
            
            case STATE_CLOSING:
                motor_reverse_one_tick(motor_delay);
                break;
            
            default:
                motor_stop();
        }
        //vTaskDelay(motor_delay*4 / portTICK_PERIOD_MS);
    }
}

static void task_ReadSwitches(void* arg)
{
    printf("Starting ReadSwitches() task...\n");
    for(;;){
        // Check upper limit switch
        uint8_t upper_sw_status = gpio_get_level(GPIO_SW_UPPER);
        uint8_t lower_sw_status = gpio_get_level(GPIO_SW_LOWER);
        uint8_t control_sw_A = gpio_get_level(GPIO_SW_CTL_A);
        uint8_t control_sw_B = gpio_get_level(GPIO_SW_CTL_B);

        if(xSemaphoreTake(xSemaphore,100/portTICK_PERIOD_MS))
        {
            state.lower_limit_switch = lower_sw_status;
            state.upper_limit_switch = upper_sw_status;
            if(control_sw_A == 0)state.controller = CTRL_RAISE; 
            if(control_sw_A != 0)state.controller = CTRL_LOWER; 
            //if((control_sw_B == 1) && (control_sw_B == 1))state.controller = CTRL_AUTO;
            xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
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

static void task_ExecuteStateMachine(void* arg)
{
    printf("Starting State Machine...\n");
    struct machine_state local_state;
    for(;;)
    {
        // Get state
        if(xSemaphoreTake(xSemaphore,100/portTICK_PERIOD_MS)) {
            local_state = state;
            // Check Mode
            if(local_state.controller == CTRL_AUTO) {
                //TO-DO: implement autoStateMachine();
            } else {
                manualStateMachine(&state);
            }
            print_state(&state);
            xSemaphoreGive(xSemaphore);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void manualStateMachine(struct machine_state *s)
{
    switch(s->active_state)
    {
        case STATE_OPEN:
        if(s->controller == CTRL_LOWER)s->active_state = STATE_CLOSING;
        break;

        case STATE_CLOSING:
        {
            if(s->lower_limit_switch == 0)s->active_state = STATE_CLOSED;
            if(s->controller == CTRL_RAISE)s->active_state = STATE_OPENING;
            break;
        }

        case STATE_CLOSED:
        if(s->controller == CTRL_RAISE)s->active_state = STATE_OPENING;
        break;

        case STATE_OPENING:
        {
            if(s->upper_limit_switch == 0)s->active_state = STATE_OPEN;
            if(s->controller == CTRL_LOWER)s->active_state = STATE_CLOSING;
            break;
        }

        default:
        //printf("Error! Unknown state\n");
    }
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    printf("arg: %ld\n",gpio_num);
}

void app_main(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_STEPPER_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    //enable interrupt
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_SW_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_SW_CTL_A, gpio_isr_handler, (void*) GPIO_SW_CTL_A);
    gpio_isr_handler_add(GPIO_SW_CTL_B, gpio_isr_handler, (void*) GPIO_SW_CTL_B);

    printf("Minimum free heap size: %"PRIu32" bytes\n", esp_get_minimum_free_heap_size());

    vSemaphoreCreateBinary( xSemaphore );
    if( xSemaphore == NULL )
    {
        printf("Error creating semaphore\n");
    }
    state.active_state = STATE_OPENING;
    xTaskCreate(&task_RunMotor, "RunMotor", 2048, NULL, 5, NULL);
    xTaskCreate(&task_ReadSwitches, "ReadSwitches", 2048, NULL, 5, NULL);
    xTaskCreate(&task_ExecuteStateMachine, "ExecuteStateMachine", 2048, NULL, 5, NULL);
}
