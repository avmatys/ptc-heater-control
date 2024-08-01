# PTC Heater Controller 

## Important note
This project was not tested in the real car.
If you have any comments regarding circuit design and source code, please contact me matys.andrey@gmail.com and I will improve them.
This is my first AVR project and my first created circuit.
Thank you.

## About the project
Control a "stupid" PTC heater. Developed for the VAG PQ46 (VW Passat B6/7/CC, Skoda Superb) platform. 
Platform A5 (PQ35) can have the same values and this project probably can be used too.

## Project details
### Input
- Voltage of the potentiometer 1 and 2 (V158 and V159) - up-to 4.7V. 
- PWM from heater electric motor - frequency 25KHz, up-to 14.7V
- Battery voltage - up-to 14.7V
### Processor
Atmega44a
### Logic
Processor enables outputs PA4-PA6 (relays) and PA7 (led) if:
- Button was clicked
- Input conditions were met
Relays are enabled in a sequence (1 -> 2 -> 3) with some delay.
Relays are disabled in a sequence (3 -> 2 -> 1).
Processor checks input conditions on every timer interruption. Relays are disabled if conditions are not met. 
### Input processing
Potentiometer signals are converted using ADC and compared with a min value.
PWM signal is transformed using RC circuit.

## Open points
- Process PWM signal in the processor
- Disable relay one by one if voltage decreases

