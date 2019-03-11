# AMX
AMX is an arduino-compatible audio and motion datalogger board (designed for Teensy 3.x)

More information is at http://loggerhead.com

### Converting AMX to wav and csv files:
- Use AMX2WAV https://github.com/loggerhead-instruments/amx/tree/master/VisualStudio/AMX2WAV/x64/Release

### Files:
- amx: Main control and sensor recording
- cmd.pde: Reads recording settings from a script file
- ISL29125: RGB light sensor interface
- MPU9250: Invensense IMU sensor interface
- wav.h: wav header
- rms: rms calculation


### Sensors:
- Accelerometer, Gyroscope, Magnetometer (MPU9250)
- Pressure/Temperature (MS5803)
- RGB light (ISL29125)
- Burn wire release: controls a FET switch to ground which can be used to corrode a stainless steel wire in seawater

### Software supports:
- duty cycle recording
- display
- button input for recording setup
- writing to microSD

## Updating Firmware

1.	Install Teensyduino from https://www.pjrc.com/teensy/loader.html
2.	Get latest hex file from appropriate repository
	https://github.com/loggerhead-instruments/amx/tree/master/hex
	To save file, right click on the filename and choose 'Save link as'
3.	Connect microUSB cable to small board on device
4.	Run the Teensy Loader program.
5.	From File Menu, select Open HEX File
6.  Turn on power to board
7.	Press the PGM button on the board next to the microSD slot, or from Operation menu, select Program (you may have to turn off automatic mode).

To check whether the firmware has been updated, collect some data, and check the log files for the version date of the firmware.
