IMU Code for Kalmann Filtered Orientation
============================================

This is Arduino code which can be uploaded to the Atmega
microcontroller on the Sparkfun Razor IMU. It gets the sensor
readings from the IMU, calculates the yaw, pitch and roll,
applies a Kalmann Filter for better accuracy and streams the
filtered data over the serial port.

The calib_files folder contains the calibration files for all
the IMUs.