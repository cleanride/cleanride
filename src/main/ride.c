#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/atomic.h"
#include "common/maths.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"

#include "rx/rx.h"

#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/escservo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/pitotmeter.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_profile.h"
#include "config/config_master.h"

void hello()
{
    /* Init L3GD20 sensor 
    if (TM_L3GD20_Init(TM_L3GD20_Scale_2000) != TM_L3GD20_Result_Ok) {
        // Sensor error 
        TM_ILI9341_Puts(10, 100, "Sensor ERROR!", &TM_Font_11x18, 0x0000, ILI9341_COLOR_RED);

        while (1);
    }

    while (1) {
        // Read data 
        TM_L3GD20_Read(&L3GD20_Data);
        // Display data on LCD 
        sprintf(buffer, "X rotation: %4d", L3GD20_Data.X);
        TM_ILI9341_Puts(10, 100, buffer, &TM_Font_11x18, 0x0000, ILI9341_COLOR_RED);
        sprintf(buffer, "Y rotation: %4d", L3GD20_Data.Y);
        TM_ILI9341_Puts(10, 122, buffer, &TM_Font_11x18, 0x0000, ILI9341_COLOR_RED);
        sprintf(buffer, "Z rotation: %4d", L3GD20_Data.Z);
        TM_ILI9341_Puts(10, 144, buffer, &TM_Font_11x18, 0x0000, ILI9341_COLOR_RED);
    }*/
    TM_L3GD20_Init();
}
#define READ_CMD               ((uint8_t)0x80)
#define MULTIPLEBYTE_CMD       ((uint8_t)0x40)
#define DUMMY_BYTE             ((uint8_t)0x00)

#define CTRL_REG1_ADDR         0x20
#define CTRL_REG4_ADDR         0x23
#define CTRL_REG5_ADDR         0x24
#define OUT_TEMP_ADDR          0x26
#define OUT_X_L_ADDR           0x28

#define MODE_ACTIVE                   ((uint8_t)0x08)

#define OUTPUT_DATARATE_1             ((uint8_t)0x00)
#define OUTPUT_DATARATE_2             ((uint8_t)0x40)
#define OUTPUT_DATARATE_3             ((uint8_t)0x80)
#define OUTPUT_DATARATE_4             ((uint8_t)0xC0)

#define AXES_ENABLE                   ((uint8_t)0x07)

#define BANDWIDTH_1                   ((uint8_t)0x00)
#define BANDWIDTH_2                   ((uint8_t)0x10)
#define BANDWIDTH_3                   ((uint8_t)0x20)
#define BANDWIDTH_4                   ((uint8_t)0x30)

#define FULLSCALE_250                 ((uint8_t)0x00)
#define FULLSCALE_500                 ((uint8_t)0x10)
#define FULLSCALE_2000                ((uint8_t)0x20)

#define BLOCK_DATA_UPDATE_CONTINUOUS  ((uint8_t)0x00)

#define BLE_MSB	                      ((uint8_t)0x40)

#define BOOT                          ((uint8_t)0x80)
void TM_L3GD20_Init() {

    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);


    GPIO_InitStructure.GPIO_Pin = Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_SetBits(GPIOC, Pin_1);

    spiSetDivisor(SPI5, SPI_9MHZ_CLOCK_DIVIDER);

    GPIO_ResetBits(GPIOC, Pin_1);

    spiTransferByte(SPI5, CTRL_REG5_ADDR);
    spiTransferByte(SPI5, BOOT);

    GPIO_SetBits(GPIOC, Pin_1);

    delayMicroseconds(100);

    GPIO_ResetBits(GPIOC, Pin_1);

    spiTransferByte(SPI5, CTRL_REG1_ADDR);

    spiTransferByte(SPI5, MODE_ACTIVE | OUTPUT_DATARATE_3 | AXES_ENABLE | BANDWIDTH_3);
    //spiTransferByte(SPI5, MODE_ACTIVE | OUTPUT_DATARATE_4 | AXES_ENABLE | BANDWIDTH_4);

    GPIO_SetBits(GPIOC, Pin_1);

    delayMicroseconds(1);

    GPIO_ResetBits(GPIOC, Pin_1);

    spiTransferByte(SPI5, CTRL_REG4_ADDR);
    spiTransferByte(SPI5, BLOCK_DATA_UPDATE_CONTINUOUS | BLE_MSB | FULLSCALE_2000);

    GPIO_SetBits(GPIOC, Pin_1);

    delay(100);
/*
    GPIO_SetPinLow(GPIOC, GPIO_PIN_1);
    GPIO_SetPinHigh(GPIOC, GPIO_PIN_1);
   
    // Init SPI
    TM_SPI_Init(L3GD20_SPI, L3GD20_SPI_PINSPACK);
    // Check if sensor is L3GD20 
    if (TM_L3GD20_INT_ReadSPI(L3GD20_REG_WHO_AM_I) != L3GD20_WHO_AM_I) {
        /* Sensor connected is not L3GD20 
        return TM_L3GD20_Result_Error;
    }

    // Enable L3GD20 Power bit 
    TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG1, 0xFF);

    // Set L3GD20 scale 
    if (scale == TM_L3GD20_Scale_250) {
        TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG4, 0x00);
    }
    else if (scale == TM_L3GD20_Scale_500) {
        TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG4, 0x10);
    }
    else if (scale == TM_L3GD20_Scale_2000) {
        TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG4, 0x20);
    }

    // Save scale 
    TM_L3GD20_INT_Scale = scale;

    // Set high-pass filter settings 
    TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG2, 0x00);

    // Enable high-pass filter 
    TM_L3GD20_INT_WriteSPI(L3GD20_REG_CTRL_REG5, 0x10);*/
}