
#ifndef ESP32_HOME_CTL_ETH_ESPIDF_EXEC_H
#define ESP32_HOME_CTL_ETH_ESPIDF_EXEC_H

#include <esp_err.h>

#define TIMER_DIVIDER         (APB_CLK_FREQ / 1000000) //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER) // 1 second

#define CTRL_DIRSETUP 0b00000001
#define CTRL_ACCEL    0b00000010
#define CTRL_PWMFREQ  0b00000100
#define CTRL_READY    0b01000000
#define CTRL_ENABLE   0b10000000

#define IO_00 0b00000001
#define IO_01 0b00000010
#define IO_02 0b00000100
#define IO_03 0b00001000
#define IO_04 0b00010000
#define IO_05 0b00100000
#define IO_06 0b01000000
#define IO_07 0b10000000

#define STEP_0_PIN 12
#define STEP_0_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT12)
#define STEP_0_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT12)
#define DIR_0_PIN 13
#define DIR_0_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT13)
#define DIR_0_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT13)

#define STEP_1_PIN 16
#define STEP_1_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT16)
#define STEP_1_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT16)
#define DIR_1_PIN 17
#define DIR_1_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT17)
#define DIR_1_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT17)

#define STEP_2_PIN 21
#define STEP_2_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT21)
#define STEP_2_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT21)
#define DIR_2_PIN 22
#define DIR_2_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT22)
#define DIR_2_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT22)

#define OUT_00_PIN 2
#define OUT_00_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT2)
#define OUT_00_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT2)
#define OUT_01_PIN 4
#define OUT_01_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT4)
#define OUT_01_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT4)
#define OUT_02_PIN 25
#define OUT_02_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT25)
#define OUT_02_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT25)
#define OUT_03_PIN 1
#define OUT_03_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT1)
#define OUT_03_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT1)
#define OUT_04_PIN 14
#define OUT_04_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT14)
#define OUT_04_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT14)
#define OUT_05_PIN 15
#define OUT_05_H REG_WRITE(GPIO_OUT_W1TS_REG, BIT15)
#define OUT_05_L REG_WRITE(GPIO_OUT_W1TC_REG, BIT15)

#define IN_00_PIN 26
#define IN_00 REG_READ(GPIO_IN_REG) & BIT26 //26
#define IN_01_PIN 27
#define IN_01 REG_READ(GPIO_IN_REG) & BIT27 //27
#define IN_02_PIN 32
#define IN_02 REG_READ(GPIO_IN1_REG) & BIT0 //32 -32
#define IN_03_PIN 33
#define IN_03 REG_READ(GPIO_IN1_REG) & BIT1 //33 -32
#define IN_04_PIN 34
#define IN_04 REG_READ(GPIO_IN1_REG) & BIT2 //34 -32
#define IN_05_PIN 35
#define IN_05 REG_READ(GPIO_IN1_REG) & BIT3 //35 -32
#define IN_06_PIN 36
#define IN_06 REG_READ(GPIO_IN1_REG) & BIT4 //36 -32
#define IN_07_PIN 39
#define IN_07 REG_READ(GPIO_IN1_REG) & BIT7 //39 -32

typedef struct {
    uint8_t control;
    uint8_t io;
    uint16_t pwm[6];
    int32_t pos[3];
    float vel[3];
} cmdPacket; // and then goes uint8_t checksum

typedef struct {
    uint8_t control;
    uint8_t io;
    int32_t pos[3];
    float vel[3];
} fbPacket;

esp_err_t processUdpPacket(char *packetBuffer, int len);
esp_err_t initExecutor();
void control_loop();

#endif //ESP32_HOME_CTL_ETH_ESPIDF_EXEC_H
