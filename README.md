# EXPERIMENT-02-Interfacing-Multiple-Switches-for-LED-Control-Using-MicroPython


 
## NAME: SOUNDARYA J

## DEPARTMENT: IT

## ROLL NO: 212223220108

## DATE OF EXPERIMENT: 12.02.2026

## AIM

To interface multiple switches with the Raspberry Pi Pico and control LEDs using MicroPython.

## APPARATUS REQUIRED

Raspberry Pi Pico

2 Push Button Switches

2 LEDs (Light Emitting Diodes)

330Ω Resistors

Breadboard

Jumper Wires

USB Cable

## THEORY



FIGURE-01: RASPBERRY PI PICO PINOUT DIAGRAM

Raspberry Pi Pico is a microcontroller board based on the RP2040 chip. It supports MicroPython, making it suitable for IoT and embedded applications. The Raspberry Pi Pico is a compact microcontroller board featuring a 40-pin layout, including power, ground, GPIO, and communication interface pins. It operates with a dual-core ARM Cortex-M0+ processor and supports MicroPython and C/C++ programming.

The power pins include VBUS (5V from USB), VSYS (1.8V to 5.5V input), 3V3(OUT) (regulated 3.3V output), and multiple ground (GND) connections. The board offers 26 multi-purpose GPIO pins (GP0 to GP28), which can be used for digital input, output, PWM, and communication interfaces such as I2C, SPI, and UART. It also features three analog-to-digital converter (ADC) pins (GP26, GP27, GP28), used for reading analog sensor values, along with an ADC_VREF pin to set the reference voltage.

For communication, I2C (SDA, SCL), SPI (MOSI, MISO, SCK), and UART (TX, RX) interfaces are mapped across different GPIO pins, allowing seamless connectivity with sensors and peripherals. All GPIO pins support PWM (Pulse Width Modulation), making it useful for motor control, LED brightness adjustment, and sound applications. The BOOTSEL button enables USB mass storage mode for firmware flashing, while the DEBUG pins (SWD interface) provide debugging capabilities. With its low power consumption, flexible GPIO options, and rich interface support, the Raspberry Pi Pico is widely used for IoT, embedded systems, robotics, and automation projects.

WORKING PRINCIPLE

The switches are connected as inputs to GPIO pins of the Pico.

The LEDs are connected as outputs.

A MicroPython script reads the switch states and controls the LEDs accordingly.

### CIRCUIT DIAGRAM
 ![image](https://github.com/user-attachments/assets/1c7234b9-5041-4156-94b8-0b846adb6b8e)
    
Figure-01 circuit diagram of digital input interface 


Connect switch 1 to GP2 and switch 2 to GP3.

Connect LED 1 to GP14 via a 330Ω resistor.

Connect LED 2 to GP17 via a 330Ω resistor.

Connect the other terminals of the switches to GND.

## PROGRAM 2A:
```
from machine import Pin
from time import sleep

switch1 = Pin(1, Pin.IN, Pin.PULL_UP)
switch2 = Pin(27, Pin.IN, Pin.PULL_UP)

led1 = Pin(14, Pin.OUT)
led2 = Pin(17, Pin.OUT)

while True:
    sw1_state = switch1.value()
    sw2_state = switch2.value()

    print("Switch 1 state:", sw1_state)
    print("Switch 2 state:", sw2_state)

    led1.value(0)
    led2.value(0)

    if sw1_state == 1 and sw2_state == 1:
        led1.value(0)
        led2.value(0)

    elif sw1_state == 0:   # pressed (PULL_UP → pressed = 0)
        led1.value(1)
        sleep(0.5)
        led1.value(0)

    elif sw2_state == 0:   # pressed
        led2.value(1)
        sleep(0.5)
        led2.value(0)

    sleep(0.1)
```
## CIRCUIT DIAGRAM:
<img width="826" height="587" alt="Screenshot 2026-02-11 105520" src="https://github.com/user-attachments/assets/39290b8c-c3b0-4e4d-9f2f-020f2a1d4eb5" />

## OUTPUT 2A
## STATE 0 0
<img width="1919" height="906" alt="Screenshot 2026-02-11 112537" src="https://github.com/user-attachments/assets/efda920b-a57f-48d2-855b-b8a14866707f" />

## STATE 1 0
<img width="1919" height="907" alt="Screenshot 2026-02-11 112553" src="https://github.com/user-attachments/assets/e2172460-1280-4612-a517-32ea53e062b3" />

## STATE 0 1
<img width="1919" height="890" alt="Screenshot 2026-02-11 112607" src="https://github.com/user-attachments/assets/f2f81efc-0461-4282-902b-b52efce38887" />

## STATE 1 1
<img width="1919" height="906" alt="Screenshot 2026-02-11 112622" src="https://github.com/user-attachments/assets/39cc1bd2-a537-49fe-abb3-6e7681503815" />

## ## PROGRAM 2B:
```
from machine import Pin
import time

print("Pi Pico Simulation")

# LED Pins (as per image)
purple_led = Pin(0, Pin.OUT)
buzzer = Pin(1, Pin.OUT)
yellow_led = Pin(14, Pin.OUT)
red_led = Pin(16, Pin.OUT)

while True:
    
    # Purple LED
    purple_led.value(1)
    print("Purple LED ON")
    time.sleep(1)
    purple_led.value(0)
    print("Purple LED OFF")
    time.sleep(1)

    # Yellow LED
    yellow_led.value(1)
    print("Yellow LED ON")
    time.sleep(1)
    yellow_led.value(0)
    print("Yellow LED OFF")
    time.sleep(1)

    # Red LED
    red_led.value(1)
    print("Red LED ON")
    time.sleep(1)
    red_led.value(0)
    print("Red LED OFF")
    time.sleep(1)

    # Buzzer
    buzzer.value(1)
    print("Buzzer ON")
    time.sleep(1)
    buzzer.value(0)
    print("Buzzer OFF")
    time.sleep(1)
```
## OUTPUT 2B:



## RESULTS

The multiple switches connected to the Raspberry Pi Pico successfully controlled the LEDs based on their states, confirming the proper interfacing of digital inputs and outputs.

