# RC-bicycle
#ABSTRACT
In a world where technology is shaping the way we move, our project aims to bring a fresh
perspective to personal mobility. At the heart of our endeavor is the development of a bicycle
robot—an innovative blend of robotics, sensor tech, and wireless connectivity.
Our project is driven by a simple yet powerful vision: to create a bicycle that goes beyond
traditional limitations. By leveraging cutting-edge advancements, we're crafting a vehicle that
seamlessly integrates into modern urban life, offering enhanced convenience and control.
With a focus on simplicity and accessibility, our bicycle robot will revolutionize how people
navigate their surroundings. Through careful design and the use of user-friendly technology,
we're striving to redefine the concept of personal mobility.
As we embark on this journey, our goal remains crystal clear: to pioneer a new era of urban
transportation solutions that blur the lines between man and machine. By embracing
innovation and quality, we're shaping a future where exploration knows no bounds and
mobility becomes an effortless extension of everyday life.

INTRODUCTION
A remote-controlled device refers to any electronic or mechanical system that can be operated
or controlled from a distance, typically using a remote control unit or a wireless interface.
These devices leverage various technologies to enable remote operation, allowing users to
manipulate their functions, settings, or movements without direct physical contact.
Remote control connection can be made using the internet, Bluetooth, infrared (IR) light, and
radio signals. It can be used for devices like TVs, house appliances, cars, drones, vehicles, etc.
Remote control's main purpose is to make life easier.
Remote-controlled vehicles are used for scientific purposes, space probes, submarines, law
enforcement, military engagements, and entertainment. Remote-controlled bicycles are one of
them and are mostly used for entertainment.
Bicycles have long been a popular mode of transportation and recreation, offering benefits
such as eco-friendliness, health promotion, and cost-effectiveness. However, traditional
bicycles rely solely on manual input from the rider, limiting their potential for automation and
remote operation. With the advent of IoT (Internet of Things) technologies and
microcontrollers, bicycles can be enhanced with remote control capabilities, thereby opening
up new possibilities for convenience, safety, and functionality.
In a remote-controlled bicycle robot, the most important part is maintaining balance. It can be
provided by constructing the bike in a symmetrical form, using sensors to understand the tilt
of the bicycle and good steering control.

FEASIBILITY STUDY
To make this project there are three basic parts we need to do. The balance provider, velocity
supply and remote control.
We will provide the balance by using a Gyro Sensor. We decided to use MPU6050. This
sensor will determine the position of the bicycle. By connecting this sensor to a servo motor,
we can provide the motion of the steering wheel. We need to place the MPU6050 that it lies
the flat on the chassis of the bicycle, with this way it can find the data in the balance position.
For the second part we need to use a DC motor to provide the motion of the bicycle. This
motor will just connected to the back wheel. The front wheel for the direction and the back
one is for the power supply. We will connect the DC motor to the back wheel and we need to
connect a driver card for it to work. For not to prevent the balance of the bicycle we need to
choose as small as possible the DC motor and driver.
For the remote control we decided to use HC05 Bluetooth Serial Module Card. It provide
remote control from the mobile device.
To make all of them work we need to connect all of the parts to a microcontroller. We decided
to use Wemos D1 Mini which is also called as ESP32.

![image](https://github.com/evrmgzm/RC-bicycle/assets/97483789/c3653eec-a11a-4844-b56d-fdb8ceefa963)

APPLICATION

Introducing the Components

ESP32

We have used NodeMCU development board based on ESP32, ESP-WROOM-32, as the microcontroller of the circuit. ESP32 based modules have integrated flash memories and hence provide effective solutions for Wi-Fi and Bluetooth based connectivity applications[1]. They can also be configured for different applications such as smart home applications, industrial automation, wearable devices, IP camera and intelligent agriculture, mostly suitable for IoT applications. NodeMCU board uses ESP32 as processor and Wi-Fi module, has 32Mbit built-in flash and onboard PCB antenna. Peripheral interface options include UART, GPIO, ADC, DAC, SDIO, PWM, I2C, I2S. The power supply works with 5V, and the logic level is 3.3V. It uses Bluetooth 4.3 and works in the frequency range between 2400MHz- 2483,5MHz. The pinout of the microcontroller is given in Figure 1.

![image](https://github.com/evrmgzm/RC-bicycle/assets/97483789/1617edb8-b03c-48b7-abc7-776ebf50c206)

MPU-6050

The MPU6050 is an IMU (Inertial Estimation Unit) sensor. IMU sensors are used in a wide variety of applications such as mobile phones, tablets, satellites, spacecraft, drones, UAVs, robotics, and many more. They are used for motion tracking, orientation and position detection, flight control, etc.
The MPU6050 is a Micro-Electro-Mechanical Systems (MEMS) that consists of a 3-axis Accelerometer and 3-axis Gyroscope inside it meaning that this helps us to measure acceleration, velocity, orientation, displacement and many other motion-related parameters of a system or object. This module also has a (DMP) Digital Motion Processor inside it which is powerful enough to perform complex calculations and thus free up the work for Microcontroller.
The working principle is based on the Coriolis Effect. The Coriolis Effect states that when a mass moves in a particular direction with velocity and an external angular motion is applied to it, a force is generated and that causes a perpendicular displacement of the mass. The force that is generated is called the Coriolis Force and this phenomenon is known as the Coriolis Effect. [3]. The rate of displacement will be directly related to the angular motion applied. The Coriolis effect causes a vibration when the gyros are rotated about any of the axes. These vibrations are picked up by the capacitor. The signal produced is then amplified, demodulated, and filtered to produce a voltage that is proportional to the angular rate. This voltage is then digitized using ADC’s. The DMP present on MPU6050 offloads the computation of motion- sensing algorithms from the host processor. DMP acquires data from all the sensors and stores the computed values in its data registers or in FIFO. FIFO can be accessed through the serial interface. Using AD0 pin more than one MPU6050 module can be interfaced with a microprocessor. MPU6050 can be used easily with Arduino, as MPU6050 has well- documented libraries available.[4] The pinout of the MPU6050 is given in Figure 2.

![image](https://github.com/evrmgzm/RC-bicycle/assets/97483789/d6025ef7-0eb9-47aa-87ac-276356a160e1)

SG 90 Servo Motor

The SG90 Servo Motor is an actuator that produces a rotary motşon approximately from 0˚ to 180˚. Servo motors in general are used in many applications such as robotic arms, steering wheels, camera gimbals, etc. Servo motors have three wires: power, ground, and signal. The power wire is red, should be connected to the power supply, usually 5V. The ground wire is usually black, or brown should be connected to the ground. The signal wire is usually yellow, or orange should be connected to a PWM (Pulse Width Modulation) on the board.
![image](https://github.com/evrmgzm/RC-bicycle/assets/97483789/722688f5-fcd5-4009-98ad-d18c6fb54747)


The angle is determined by the duration of a pulse that is applied to the control wire. This is called Pulse Width Modulation. The length of the pulse will determine how far the motor turns. The microcontroller sends out PWM signals to the servo, and then the embedded board on the servo receives the signals through the signal pin and the controls the motor inside to turn. As a result, the motor drives the gear system and then rotates the shaft after deceleration. The shaft and the potentiometer of the servo are connected. When the shaft rotates, it drives the potentiometer, so the potentiometer outputs a signal to the embedded board. Then the
 
board determines the direction and the speed of the rotation based on the current position, so it can stop exactly at the right position as defined and hold there.

![image](https://github.com/evrmgzm/RC-bicycle/assets/97483789/8246cb12-a1c7-4c1b-9d21-36f98d3fe4a1)

Position "0" (1.5 ms pulse) is middle, "90" (~2 ms pulse) is all the way to the left. ms pulse) is middle, "90" (~2 ms pulse) is all the way to the right, " the left.

![image](https://github.com/evrmgzm/RC-bicycle/assets/97483789/ff4b0eb9-269c-453b-9b7d-75bd0cd2bc18)


DRV8833

To control the speed and direction of the DC motor, we are going to use DRV-8833. The DRV8833 features two NMOS H-bridge drivers, enabling it to control two DC brush motors, a bipolar stepper motor, solenoids, and other inductive loads. The driver supply voltage is 2.7 to 10.8V and the allowable continuous current is 1.5A and the maximum allowable current is 2A. Some common applications are: POS printers, toys, robotics, office automation systems.
To change the direction of rotation of a DC motor you need to change the polarity of the supply. A common technique to do that is by using an H-bridge motor driver. An H-bridge motor driver consists of four switches (usually MOSFETs) arranged in a certain formation and the motor is connected to the center of the arrangement to form the H-like structure, by closing/enabling two opposite switches we can change the direction of the flowing current thus changing the direction of rotation.
![image](https://github.com/evrmgzm/RC-bicycle/assets/97483789/61c79335-30c1-4ae9-b53d-0ac993e7a017)


The pinout of the DRV-8833 is given in Figure 6. This module has 16 pins:

•	VM: Motor voltage
•	GND: Ground – three grounds connected to each other
•	AO1: Positive end for motor A
•	AO2: Negative end for motor A
•	BO1: Positive end for motor B
•	BO2: Negative end for motor B
•	AIN1: Control signal for motor A
 
•	AIN2: Control signal for motor A
•	BIN1: Control signal for motor B
•	BIN2: Control signal for motor B
•	STBY: To active Standby mode, it should be HIGH.
•	NC: Not used
Different modes of the control signals provide different performance modes for DC motors. Different modes are given in Table 1.

	AIN1 / (BIN1)	AIN2 / (BIN2)
Forward Direction	HIGH	LOW
Reverse Direction	LOW	HIGH
Brake / Stopped	LOW	LOW
Brake / Stopped	HIGH	HIGH
Table 1.






