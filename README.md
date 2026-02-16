# Sotong Game

* **Course:** EE2028 Microcontroller Progamming and Interfacing
* **Project Description:** This is an interactive game implemented on the B-L4S5I-IOT01A microcontroller board which uses
  the STM32L4S5VIT6 SoC. We configured GPIO pins for on-board peripherals (gyroscope, accelerometer, temperature sensor, etc.),
  on-chip peripherals (I2C, UART, EXTI for external interrupts), and wrote to registers using HAL(On-chip peripherals), BSP(On-board peripherals)
  and CMSIS (Core Peripherals) libraries. The game logic was written in non-blocking C code, which utilized timers and flags.
  We also implemented an additional gyroscope interrupt logic for detecting tilt movement, OLED for interactive low-power animation display,
  and a buzzer for generating tones.
* Check out [EE2028 - Sotong Game]([https://docs.github.com](https://sites.google.com/view/liewkangzhen/home/ee2028-project)) to see the physical demo and gain more information. Cheers!
