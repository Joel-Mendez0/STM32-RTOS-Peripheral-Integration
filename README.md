
# STM32 Peripheral Integration with RTOS

This project demonstrates the integration of various peripherals (servo motors, ultrasonic sensors, motion sensors, etc.) with an STM32 microcontroller, using Real-Time Operating System (RTOS) to manage multiple tasks concurrently.

## Features

- **Servo Motor Control**: Uses PWM to control the position of a servo motor.
- **Ultrasonic Sensor**: Measures distance using an ultrasonic sensor and displays the result on an LCD.
- **Motion Detection**: Uses ADC to read motion sensor data.
- **LCD Display**: Continuously updates and displays sensor data on a 16x2 LCD.
- **RTOS**: Implements FreeRTOS to manage multiple threads efficiently.

## Project Structure

- **Servo Control**: The `UPDATESERVO` thread manages the servo motor's position using PWM.
- **Ultrasonic Sensor**: The `UPDATEDISTANCE` thread handles ultrasonic sensor data collection, computes distance, and displays it on the LCD.
- **Motion Sensor**: The `UPDATEMOTION` thread reads ADC values from the motion sensor.
- **LCD Display**: The `UPDATELCD` thread updates the LCD screen with real-time data from the sensors.
- **LED Blinkers**: `STARTBLINK0` and `STARTBLINK1` threads toggle an onboard LED at different intervals as a simple demonstration of RTOS task management.

## Code Snippets

### Servo Motor Control

```c
void UPDATESERVO(void *argument) {
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
    for(;;) {
        uint32_t startTick = HAL_GetTick();
        while((HAL_GetTick() - startTick) < 5000);
        if(servo_flag == 0) {
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 900);
            servo_flag = 1;
        } else {
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 2100);
            servo_flag = 0;
        }
    }
}
```

### Ultrasonic Sensor Measurement

```c
void UPDATEDISTANCE(void *argument) {
    for(;;) {
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
        usDelay(3);
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
        usDelay(10);
        HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
        uint32_t startTick = HAL_GetTick();
        do {
            if(icFlag) break;
        } while((HAL_GetTick() - startTick) < 500);
        icFlag = 0;
        HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
        DISTANCE = (edge2Time > edge1Time) ? ((edge2Time - edge1Time) + 0.0f) * speedofSound : 0.0f;
        osDelay(2000);
    }
}
```

### LCD Update

```c
void UPDATELCD(void *argument) {
    lcd_init();
    for(;;) {
        lcd_clear();
        sprintf(LCD_BUF_0, "ADC RAW: %u", ADC_MOTION);
        sprintf(LCD_BUF_1, "DIS: %06.2f cm", DISTANCE);
        lcd_send_string(LCD_BUF_0);
        lcd_put_cur(1, 0);
        lcd_send_string(LCD_BUF_1);
        osDelay(1000);
    }
}
```

### Motion Detection with ADC

```c
void UPDATEMOTION(void *argument) {
    for(;;) {
        HAL_ADC_Start_IT(&hadc1);
        sprintf(LCD_BUF_0, "ADC RAW: %u", ADC_MOTION);
        osDelay(500);
    }
}
```

## Requirements

- **Hardware**:
  - STM32 microcontroller (e.g., STM32F4)
  - Servo motor
  - Ultrasonic sensor (HC-SR04)
  - Motion sensor
  - 16x2 LCD Display
  - LED (for blink demonstration)

- **Software**:
  - STM32CubeMX for peripheral initialization
  - STM32CubeIDE for coding and debugging
  - FreeRTOS for task management

## Getting Started

1. Clone the repository.
2. Open the project in STM32CubeIDE.
3. Connect the peripherals (servo motor, ultrasonic sensor, etc.) to the STM32 as per the pin configuration in the code.
4. Build and flash the project onto the STM32.
5. Observe the output on the LCD screen and servo movement, along with LED toggling as controlled by RTOS.

## How It Works

- **Task Management**: FreeRTOS handles concurrent tasks, each assigned to manage a peripheral or function, such as reading sensor data or controlling the servo.
- **Timers & Interrupts**: The project uses timers and interrupts for tasks like ADC conversions, capturing input signals from the ultrasonic sensor, and generating PWM for the servo motor.
- **LCD Updates**: Real-time data from the motion sensor and ultrasonic sensor are displayed on the LCD using dedicated RTOS threads.

## Contributing

Feel free to fork this repository and submit pull requests with improvements or bug fixes. Contributions are always welcome!

## License

This project is licensed under the MIT License.
