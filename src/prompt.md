Write an Arduino program with the following requirements:

1. It periodically takes temperature readings from a DHT22 sensor.
2. Let's a user modify the target temperature via a HW-040 rotary encoder whose rotation is tracked by an interrupt service routine.
3. Each click of the rotary encoder should correspond to 0.1 degrees C.
4. It displays the current temperature and target temperatures on an ST7735 display, 160x80 resolution.
5. Checks the target and current temperature periodically, and activates a relay when they differ until the current temperature matches the target.
6. It goes into low power mode if no user activity detected for some time.
7. All operations should be non-blocking so the display and rotary encoder counting is responsive.