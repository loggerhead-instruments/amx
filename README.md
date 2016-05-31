# AMX
AMX is an arduino-compatible audio and motion datalogger board (designed for Teensy 3.x)

More information is at http://loggerhead.com

Files:
- amx: Main control and sensor recording
- cmd.pde: Reads recording settings from a script file
- ISL29125: RGB light sensor interface
- MPU9250: Invensense IMU sensor interface
- wav.h: wav header
- rms: rms calculation


Sensors:
- Accelerometer, Gyroscope, Magnetometer (MPU9250)
- Pressure/Temperature (MS5803)
- RGB light (ISL29125)
- Burn wire release: controls a FET switch to ground which can be used to corrode a stainless steel wire in seawater

Software supports:
- duty cycle recording
- display
- button input for recording setup
- writing to microSD