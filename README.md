# STM32_QUAD_COPTER.   
flight controller congifuration!  
MCU:        STM32L476RG.   
ACC & GYRO: LSM6DS3.   
Magnetometer: LIS3MDL.   
  
PORTS:  
I2C1  
SCL: PB8. 
SDA: PB9. 
  
I2C1:   
Pressure and temperature: LPS25H  

Leds:   
PA5: GREEN. 
PC6: RED. 
  
Receiver inputs:  
GPIO_NO      | RECEIVER CHANNEL     |  USAGE    |
------------ |----------------------|---------- |
PC8          |    RECEIVER_CHANNEL1 | ROLL      |
PB1          |    RECEIVER_CHANNEL2 | PITCH     |
PA2          |    RECEIVER_CHANNEL3 | THROTTLE  |
PA3          |    RECEIVER_CHANNEL4 | YAW       |
  
MOTOR CONNECTIONS:                                                  
 ----------------------------------------------  
  
   | GPIO_NO  |  MOTOR_NO   |
   | ---------|  ---------- |         
   |  PA10    |     M1      |                              
   |  PA11    |     M2      |                               
   |  PB14    |     M3      |                            
   |  PC9     |     M4      |              

MOTOR LAYOUT:
-----------------------------------------------

![alt text](https://github.com/kasisripada17/STM32_QUAD_COPTER/blob/25ec00073ce909a125d3cee5a9726c821a49fc45/docs/motor_layout.png)
