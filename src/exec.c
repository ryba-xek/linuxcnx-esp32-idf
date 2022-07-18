#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <stdio.h>
#include "esp_eth.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/timer.h"
#include "exec.h"

cmdPacket cmd;
fbPacket fb;
int64_t ul_watchdog;

bool pwm_enable[OUT_PIN_COUNT] = { false/*, false, false, false, false, false*/ };
const ledc_channel_t ledc_chan[OUT_PIN_COUNT] = { LEDC_CHANNEL_0/*, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_CHANNEL_5*/ };
const int pwm_out_pin[OUT_PIN_COUNT] = { OUT_00_PIN/*, OUT_01_PIN, OUT_02_PIN, OUT_03_PIN, OUT_04_PIN, OUT_05_PIN*/ };

volatile unsigned long ul_dirSetup[3] = { 1000, 1000, 1000 }; // x 25 nanosec
volatile float f_accel_x2[3] = { 1000.0, 1000.0, 1000.0 }; // acceleration*2 step/sec2
volatile unsigned long ul_cmd_T[3];

volatile unsigned long ul_T[3] = { 0, 0, 0 };
volatile unsigned long ul_TH[3] = { 0, 0, 0 };
volatile unsigned long ul_TL[3] = { 0, 0, 0 };
volatile bool b_math[3] = { false, false, false };
volatile bool b_dirSignal[3] = { false, false, false };
volatile bool b_dirChange[3] = { false, false, false };
unsigned long ul_accelStep[3] = { 0, 0, 0 };

static const char *TAG = "executor";

void IRAM_ATTR commandHandler()
{
    ul_watchdog = esp_timer_get_time(); // new valid packet arrived -> reset watchdog
    ESP_LOGI(TAG, "cmd.control = %x, pos[0] = %d, pos[1] = %d, pos[2] = %d, vel[0] = %6.1f, vel[1] = %6.1f", cmd.control, cmd.pos[0], cmd.pos[1], cmd.pos[2], cmd.vel[0], cmd.vel[1]);
    
    // set velocity
    if (cmd.control & CTRL_READY) {
        for (int i = 0; i < 3; i++) {
            if (cmd.vel[i] > 0.0f)
                ul_cmd_T[i] = (unsigned long)(40000000.0f / cmd.vel[i]);
            else if (cmd.vel[i] < 0.0f)
                ul_cmd_T[i] = (unsigned long)(40000000.0f / -cmd.vel[i]);
            else
                ul_cmd_T[i] = 40000000ul;
            
            ESP_LOGI(TAG, "Setting %d axis velocity to %lu", i, ul_cmd_T[i]);
        }
    }

    // have READY status -> return
    if (fb.control & CTRL_READY) {
        ESP_LOGI(TAG, "have READY state, command processing finished");
        return;
    }

    // ?
    if (fb.control & (CTRL_DIRSETUP | CTRL_ACCEL | CTRL_PWMFREQ)) {
        fb.control |= CTRL_READY;
        ESP_LOGI(TAG, "setting READY state");
        return;
    }
    
    // set DIR command
    if (cmd.control & CTRL_DIRSETUP) {
        ESP_LOGI(TAG, "CTRL_DIRSETUP");
        fb.control |= CTRL_DIRSETUP;
        for (int i = 0; i < 3; i++)
            ul_dirSetup[i] = cmd.pos[i] / 25; //   25ns / timer tic

        return;
    }
    
    // set ACCEL command
    if (cmd.control & CTRL_ACCEL) {
        ESP_LOGI(TAG, "CTRL_ACCEL");
        fb.control |= CTRL_ACCEL;
        for (int i = 0; i < 3; i++)
            f_accel_x2[i] = (float) cmd.pos[i] * 2.0;

        return;
    }
    
    // set PWMFREQ command
    if (cmd.control & CTRL_PWMFREQ) {
        ESP_LOGI(TAG, "CTRL_PWMFREQ");
        fb.control |= CTRL_PWMFREQ;
        for (int i = 0; i < OUT_PIN_COUNT; i++) {
            if (cmd.pwm[i]) { // set PWM frequency
                // ledcAttachPin(pwm_pin[i], i * 2);
                // ledcSetup(i * 2, cmd.pwm[i], 10);
                // ledcWrite(i * 2, 0);
                //TODO
                // pwm_enable[i] = true;
            }/* else { // turn off PWM
                if (i == 0) {
                    pinMode(OUT_00_PIN, OUTPUT);
                    digitalWrite(OUT_00_PIN, 0);
                } else if (i == 1) {
                    pinMode(OUT_01_PIN, OUTPUT);
                    digitalWrite(OUT_01_PIN, 0);
                } else if (i == 2) {
                    pinMode(OUT_02_PIN, OUTPUT);
                    digitalWrite(OUT_02_PIN, 0);
                } else if (i == 3) {
                    pinMode(OUT_03_PIN, OUTPUT);
                    digitalWrite(OUT_03_PIN, 0);
                } else if (i == 4) {
                    pinMode(OUT_04_PIN, OUTPUT);
                    digitalWrite(OUT_04_PIN, 0);
                } else if (i == 5) {
                    pinMode(OUT_05_PIN, OUTPUT);
                    digitalWrite(OUT_05_PIN, 0);
                }
            }*/
        }

        return;
    }
}

void IRAM_ATTR readInputs()
{
    if (IN_00) 
        fb.io = IO_00;
    else
        fb.io = 0;
    
    // if (IN_01)
    //     fb.io |= IO_01;
    // if (IN_02)
    //     fb.io |= IO_02;
    // if (IN_03)
    //     fb.io |= IO_03;
    // if (IN_04)
    //     fb.io |= IO_04;
    // if (IN_05)
    //     fb.io |= IO_05;
    // if (IN_06)
    //     fb.io |= IO_06;
    // if (IN_07)
    //     fb.io |= IO_07;
}

void IRAM_ATTR outputHandler()
{
    static uint16_t last_pwm[OUT_PIN_COUNT] = { 0 /*, 0, 0, 0, 0, 0*/ };
    bool enable = cmd.control & CTRL_ENABLE;

    for (int i = 0; i < OUT_PIN_COUNT; i++) {
        bool need_set_pwm = false;
        if (pwm_enable[i]) { // output is in PWM mode
            if (enable && cmd.pwm[i] != last_pwm[i]) { // changed
                    last_pwm[i] = cmd.pwm[i];
                    need_set_pwm = true;
            }
            
            if (!enable) { //
                last_pwm[i] = 0;
                need_set_pwm = true;
            }

            if (need_set_pwm) {
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_chan[i], (uint32_t) last_pwm[i]));
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_chan[i]));
                ESP_LOGI(TAG, "Setting %d PWM output to %d", i, last_pwm[i]);
            }
        } else { // output is in discrete mode
            if (enable)
                (cmd.io & IO_00) ? OUT_00_H : OUT_00_L; //TODO
            else
                OUT_00_L;
        }
    }
}

esp_err_t processUdpPacket(char *packetBuffer, int len) {
    // check checksum
    uint8_t chksum = 71;
    for (int i = 0; i < sizeof(cmd); i++)
        chksum ^= packetBuffer[i];

    ESP_LOGI(TAG, "Checksum %x @ %d, expected: %x", packetBuffer[sizeof(cmd)], sizeof(cmd), chksum);
    if (packetBuffer[sizeof(cmd)] == chksum) {
        memcpy(&cmd, packetBuffer, sizeof(cmd));
        // ESP_LOGI(TAG, "cmd = %x", cmd.control);

        commandHandler();
        // ul_watchdog = millis();
    }

    readInputs();

    // fill feedback velocities
    for (int i = 0; i < 3; i++) {
        unsigned long ul_t = ul_T[i];
        if (ul_t) {
            fb.vel[i] = b_dirSignal[i] ? 40000000.0f / (float)ul_t : -40000000.0f / (float) ul_t;
        } else {
            fb.vel[i] = 0.0f;
        }
    }

    memcpy(packetBuffer, &fb, sizeof(fb));

    outputHandler();

    return ESP_OK;
}

static bool IRAM_ATTR onTime_0()
{
    static bool b_stepState = false;
    static bool b_dirState = false;
    
    if (b_stepState == false) { // end of period
        if (ul_T[0]) { // there is a period time so a pulse must be started
            if (!b_dirChange[0]) { //direction is good, start a new pulse (step L-H transition)
                STEP_0_H;
                timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ul_TH[0]);
                b_stepState = true;
                b_dirState ? fb.pos[0]++ : fb.pos[0]--;
                b_math[0] = true; // calculation of a new period time
            } else { // need to change direction
                if (b_dirSignal[0]) {
                    DIR_0_H;
                    b_dirState = true;
                } else {
                    DIR_0_L;
                    b_dirState = false;
                }
                timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ul_dirSetup[0]);
                b_dirChange[0] = false;
            }
        } else { // no need to step pulse
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 4000ul); // 0,1 millis scan
            b_math[0] = true; // calculation of a new period time
        }
    } else { // the middle of the period time (step H-L transition)
        STEP_0_L;
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ul_TL[0]);
        b_stepState = false;
    }

    return false; // return whether we need to yield at the end of ISR
}

static bool IRAM_ATTR onTime_1()
{
    static bool b_stepState = false;
    static bool b_dirState = false;

    if (b_stepState == false) { // end of period
        if (ul_T[1]) { // there is a period time so a pulse must be started
            if (!b_dirChange[1]) { //direction is good, start a new pulse (step L-H transition)
                STEP_1_H;
                timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, ul_TH[1]);
                b_stepState = true;
                b_dirState ? fb.pos[1]++ : fb.pos[1]--;
                b_math[1] = true; // calculation of a new period time
            } else { // need to change direction
                if (b_dirSignal[1]) {
                    DIR_1_H;
                    b_dirState = true;
                } else {
                    DIR_1_L;
                    b_dirState = false;
                }
                timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, ul_dirSetup[1]);
                b_dirChange[1] = false;
            }
        } else { // no need to step pulse
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, 4000ul); // 0,1 millis scan
            b_math[1] = true; // calculation of a new period time
        }
    } else { // the middle of the period time (step H-L transition)
        STEP_1_L;
        timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, ul_TL[1]);
        b_stepState = false;
    }

    return false; // return whether we need to yield at the end of ISR
}

static bool IRAM_ATTR onTime_2()
{
    static bool b_stepState = false;
    static bool b_dirState = false;

    if (b_stepState == false) { // end of period
        if (ul_T[2]) { // there is a period time so a pulse must be started
            if (!b_dirChange[2]) { //direction is good, start a new pulse (step L-H transition)
                STEP_2_H;
                timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, ul_TH[2]);
                b_stepState = true;
                b_dirState ? fb.pos[2]++ : fb.pos[2]--;
                b_math[2] = true; // calculation of a new period time
            } else { // need to change direction
                if (b_dirSignal[2]) {
                    DIR_2_H;
                    b_dirState = true;
                } else {
                    DIR_2_L;
                    b_dirState = false;
                }
                timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, ul_dirSetup[2]);
                b_dirChange[2] = false;
            }
        } else { // no need to step pulse
            timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 4000ul); // 0,1 millis scan
            b_math[2] = true; // calculation of a new period time
        }
    } else { // the middle of the period time (step H-L transition)
        STEP_2_L;
        timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, ul_TL[2]);
        b_stepState = false;
    }

    return false; // return whether we need to yield at the end of ISR
}

esp_err_t initExecutor() {
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_10_BIT, // Set duty resolution to 10 bits,
        .freq_hz          = 5000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    /*for (int i = 0; i < OUT_PIN_COUNT; i++) {
        // Prepare and then apply the LEDC PWM channel configuration
        ledc_channel_config_t ledc_channel = {
            .speed_mode     = LEDC_HIGH_SPEED_MODE,
            .channel        = ledc_chan[i],
            .timer_sel      = LEDC_TIMER_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = pwm_out_pin[i],
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
        pwm_enable[i] = true;
    }*/

    /* Initialize timers */
    timer_config_t tim0_config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_0, &tim0_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, onTime_0, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_0);

    timer_config_t tim1_config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_0, TIMER_1, &tim1_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_1);
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_1, onTime_1, NULL, 0);
    timer_start(TIMER_GROUP_0, TIMER_1);

    timer_config_t tim2_config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN,
    }; // default clock source is APB
    timer_init(TIMER_GROUP_1, TIMER_0, &tim2_config);
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);
    timer_isr_callback_add(TIMER_GROUP_1, TIMER_0, onTime_2, NULL, 0);
    timer_start(TIMER_GROUP_1, TIMER_0);

    gpio_pad_select_gpio(STEP_0_PIN);
    ESP_ERROR_CHECK(gpio_set_direction(STEP_0_PIN, GPIO_MODE_OUTPUT)); // TODO: GPIO_MODE_OUTPUT_OD
    gpio_pad_select_gpio(DIR_0_PIN);
    ESP_ERROR_CHECK(gpio_set_direction(DIR_0_PIN, GPIO_MODE_OUTPUT)); // TODO: GPIO_MODE_OUTPUT_OD
    gpio_pad_select_gpio(STEP_1_PIN);
    ESP_ERROR_CHECK(gpio_set_direction(STEP_1_PIN, GPIO_MODE_OUTPUT)); // TODO: GPIO_MODE_OUTPUT_OD
    gpio_pad_select_gpio(DIR_1_PIN);
    ESP_ERROR_CHECK(gpio_set_direction(DIR_1_PIN, GPIO_MODE_OUTPUT)); // TODO: GPIO_MODE_OUTPUT_OD
    gpio_pad_select_gpio(STEP_2_PIN);
    ESP_ERROR_CHECK(gpio_set_direction(STEP_2_PIN, GPIO_MODE_OUTPUT)); // TODO: GPIO_MODE_OUTPUT_OD
    gpio_pad_select_gpio(DIR_2_PIN);
    ESP_ERROR_CHECK(gpio_set_direction(DIR_2_PIN, GPIO_MODE_OUTPUT)); // TODO: GPIO_MODE_OUTPUT_OD

    ESP_ERROR_CHECK(gpio_set_direction(OUT_00_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction(IN_00_PIN, GPIO_MODE_INPUT));

    return ESP_OK;
}

float IRAM_ATTR fastInvSqrt(const float x)
{
    const float xhalf = x * 0.5f;
    union {
        float x;
        uint32_t i;
    } u = { .x = x };
    u.i = 0x5f3759df - (u.i >> 1);
    return u.x * (1.5f - xhalf * u.x * u.x);
}

void IRAM_ATTR newT(const int i) // period time calculation
{
    //ul_T[i] = 40000000.0f / sqrtf((float)(ul_accelStep[i] * ul_accel_x2[i]));
    ul_T[i] = 40000000.0f * fastInvSqrt((float) ul_accelStep[i] * f_accel_x2[i]); // fast method (>3x)
    ul_TH[i] = ul_T[i] >> 1;
    ul_TL[i] = ul_T[i] - ul_TH[i];
}

void IRAM_ATTR deceleration(const int i)
{
    if (ul_accelStep[i]) {
        ul_accelStep[i]--;
        if (ul_accelStep[i])
            newT(i);
        else
            ul_T[i] = 0;
    }
}

void IRAM_ATTR acceleration(const int i)
{
    if (cmd.control & CTRL_ENABLE) {
        ul_accelStep[i]++;
        newT(i);
    } else
        deceleration(i);
}

void IRAM_ATTR control_loop(void *pvParameters)
{   
    while (true) {
        // check packet watchdog
        int64_t watchdog_intvl = esp_timer_get_time() - ul_watchdog;
        if (watchdog_intvl > WATCHDOG_TIMEOUT) {
            //ESP_LOGI(TAG, "Command watchdog triggered [ %lld us ], shutting down all outputs", watchdog_intvl);
            fb.control = 0;
            cmd.control = 0;
            outputHandler();
            ul_watchdog = esp_timer_get_time();
        }

        for (int i = 0; i < 3; i++) {
            if (b_math[i]) {
                b_math[i] = false;
                if (!ul_accelStep[i]) { // the axis is stationary
                    long l_pos_error = cmd.pos[i] - fb.pos[i];
                    if (l_pos_error) { // if there is a position error
                        if ((l_pos_error > 0 && b_dirSignal[i] == true) || (l_pos_error < 0 && b_dirSignal[i] == false)) // the direction is good
                            acceleration(i);
                        else { // need to change direction
                            b_dirSignal[i] = l_pos_error > 0;
                            b_dirChange[i] = true;
                        }
                    }
                } else { // the axis moves
                    if ((cmd.vel[i] > 0.0f && b_dirSignal[i] == true) || (cmd.vel[i] < 0.0f && b_dirSignal[i] == false)) { // the direction is good
                        if (ul_T[i] > ul_cmd_T[i]) // the speed is low
                            acceleration(i);
                        else if (ul_T[i] < ul_cmd_T[i]) // the speed is high
                            deceleration(i);
                    } else // the direction is wrong or the target speed is zero
                        deceleration(i);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    vTaskDelete(NULL);
}