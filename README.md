# Gyro-Mouse
Creating a usb-hid device (mouse) using mpu6050 (imu sensor) and STM32Fx board using freeRTOS on Keil uVision5 ide and CubeMx software.
The accelerometer and the gyroscope values of the mpu6050 were taken and a coplementary filter was used to refine the noise in the raw values.
These refined values were then taken and thresholded to perform certain tasks like moving the mouse.
# APPLICATIONS
We can wear such a small device like the mpu6050 on our hand or forehead and threshold it to move as per our movements.
