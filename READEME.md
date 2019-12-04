# Libary to use BME280 on a STM32 NUCLEO-F767ZI via I2C.
The libary was developed using the Adafruit BME280 Arduino libary (https://github.com/adafruit/Adafruit_BME280_Library).
## Pinning
PB8: I2C1_SDA
PB9: I2C1_SCL

#How to use it
1. Create a BME280 object.
2. Call the init methode with a profile of your choise.
3. Use the different get methodes to read your wanted data. 
