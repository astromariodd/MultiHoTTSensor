MultiHoTT-Sensor
================
another telemetry module basend on the Arduino ProMini plattform and is compatible to Graupner's HoTT.
It can be used as a stand-alone telemtry module or as an extension module for the Multirotor flight-control system MultiWii, where the telemtry data are gathered via MultiWii's serial protocol, named MSP.

Current status of development
=============================
* talks to Transmitter via HoTT protocol
* emulates EAM, GAM, Vario
* also sends HoTT text based telemetry 
* reads single cell voltage with 3S-4S support
* reads it´s own baro via I2C
* talks with MultiWii over serial protocol
* reads and writes MultiWii PID and RC values
