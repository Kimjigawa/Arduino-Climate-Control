# Arduino Climate Control
Arduino Uno program design to regulate the temperature, humidity and lightning of a small controlled environment with settings parametrizable through LCD screen menu and rotary encoder.  


Available settings: Temperature (ON/OFF & target Temp), Relative Humidity (ON/OFF & target Hum), lightning schedule (ON/OFF & daily hours)

Sensors: DHT22

Devices: Arduino Uno, 4 Relays Modules, I2C TWI 1602 Serial LCD Module Display, DS3231 Real Time Clock (RTC) Module, KY-040 Rotary Encoder Module

External Libraries:  
New LiquidCrystal  (https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads)  
DS3231  (http://www.rinkydinkelectronics.com/library.php?id=73)  
SimpleDHT  (https://github.com/winlinvip/SimpleDHT/releases)
