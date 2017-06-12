# Raspberry-and-10DOF
Raspberry Pi3 + 10DOF (MPU9255+BMP280)


The program returns the data in JASON format!
Example:
{"status":"success","roll":"116.76","pitch":"-128.29","yaw":"-107.00","press":"98574","MPU_temp":"7824","BMP_temp":"4338"}

roll = 116.76 Degrees
pitch = -128.29 Degrees
yaw = -107.00 Degrees
press = 98574 Pa (or 869.86 mmHg)
MPU temperature = 78.24 Degrees
BMP temperature = 43.38 Degrees


If you want to use MYSQL:
- use makefile_withMYSQL
- Edit the file printIMU.cpp: line: "#define USE_MYSQL" and add line in end program: "MYSQL_reload();"

