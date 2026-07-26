# Delivery Robot - ESP-IDF Project

## Folder structure
```
delivery_robot/
├── CMakeLists.txt              (top-level project file)
├── components/
│   ├── main/                   app_main() - ties everything together
│   ├── motor/                  L298N driver control
│   ├── ir_ultrasonic/          4x IR line sensors + HC-SR04
│   └── color_sensor/           TCS34725 driver over I2C
```

## Building (VS Code with ESP-IDF extension, or terminal)
```
idf.py set-target esp32
idf.py build
idf.py -p <YOUR_PORT> flash monitor
```

## Before powering on - hardware checklist
1. **HC-SR04 ECHO voltage divider.** ECHO outputs 5V logic; ESP32
   GPIOs are 3.3V tolerant only. Put a voltage divider (e.g. 1k +
   2k resistors) or a logic level shifter between ECHO and GPIO19.
2. **GPIO12 boot-strap pin.** Used here for L298N IN3. If you get
   random boot failures after wiring the motor driver, move IN3
   to a different free pin (e.g. GPIO5 or GPIO17) in
   `components/motor/motor.c`.
3. **Common ground** between ESP32, L298N, and the motor battery.

## Calibration
- **Color thresholds:** uncomment the debug log line in
  `color_classify()` (`components/color_sensor/color_sensor.c`),
  flash, and watch `idf.py monitor` while holding the sensor over
  each color (and the black line, and bare floor). Adjust the
  percentage thresholds to match your sensor/lighting.
- **IR polarity:** if the robot steers away from the line instead
  of following it, flip `LINE_DETECTED_LEVEL` from `1` to `0` in
  `components/ir_ultrasonic/ir_ultrasonic.c`.

## Behavior assumption (change if this isn't your setup)
The robot ignores the *first* distinct color it sees (treated as
the entry/start marker). Once it has confirmed it's on the actual
black line, the *next* distinct color it sees is treated as the
destination and it stops for delivery. To customize this (e.g.
different actions per color, or multiple stops along one route),
edit the color-handling block in `components/main/main.c`.
