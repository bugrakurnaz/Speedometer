# Speedometer

This is a program to be run on STM32F407. It communicates via UART with a middleware, that in turn communicates with ETS2 Telemetry Server to display speed information on a 20x4 Monochrome LCD Panel.

Here are the links to these programs:
https://github.com/bugrakurnaz/Speedometer-Telemetry-Serial-Server
https://github.com/Funbit/ets2-telemetry-server

# Prerequisities

1. STM32F407G-DISC1 Board.
2. 20x4 LCD Display with I2C Expansion Card
3. UART Usb to TTL cable
4. Breadboard (optional, may be built without a breadboard, but I used one for convenience)

# PIN Connections

I2C Expansion cable:

Yellow -> 5V output of STM32
Red -> PC9 pin
Orange -> PA8 pin
White -> GND of STM32

UART Cable:

Green -> PB11 
White -> PD8
Black -> GND

# Run Configuration

Open the project in STM32CubeIde, and run. 
