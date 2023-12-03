# Bed occupancy sensor with ESP32-C6

Handles bed occupancy on two sides of the bed using two SF15-600 pressure sensors.

Uses ADC channels 2 and 3 to read the voltage and decide whether that side of the bed is occupied. 

## Configure the project

Before project configuration and build, make sure to set the correct chip target using `idf.py --preview set-target TARGET` command.

## Erase the NVRAM

Before flash it to the board, it is recommended to erase NVRAM if user doesn't want to keep the previous examples or other projects stored info using `idf.py -p PORT erase-flash`

## Build and Flash

Build the project, flash it to the board, and start the monitor tool to view the serial output by running `idf.py -p PORT flash monitor`.

(To exit the serial monitor, type ``Ctrl-]``.)
