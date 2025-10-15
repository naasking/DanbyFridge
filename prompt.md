Write an Arduino program with the following requirements:

1. It periodically takes temperature readings from a DHT22 sensor.
2. Let's a user modify the target temperature via a KY-040 rotary encoder whose rotation is tracked by an interrupt service routine.
3. Each click of the rotary encoder button should correspond to 0.1 degrees C.
4. It constantly displays the current temperature and target temperatures on an ST7789 display, and displays the target temperature as the rotary encoder is being turned, without delay.
5. Checks the target and current temperature every 10 seconds, and activates a relay when they differ until the current temperature matches the target.