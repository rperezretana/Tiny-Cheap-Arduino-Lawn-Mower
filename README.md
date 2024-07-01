# Robotic Lawn Mower
 Arduino nano simple lawnmower.
 My goal is to build a simple robotic lawnmower that costs less than $150 to build. This Robotic Lawn Mower is a robot designed to efficiently navigate and mow a 10x10 meter garden. The mower operates using IR sensors and an IR lamp for orientation, ensuring precise movement and coverage of the garden area. This project showcases a blend of hardware and software engineering, leveraging Arduino for control and Bluetooth for debugging and calibration.

## Features
Autonomous Navigation: Utilizes four IR sensors (front, back, left, right) to locate an IR lamp positioned in the southern part of the garden.
Precise Coverage: Navigates a 10x10 meter garden divided into 50cm x 50cm tiles.
Calibration Function: Establishes threshold values for boundary wire detection to ensure accurate navigation.
Bluetooth Debugging: Allows for real-time debugging and serial event reading through a Bluetooth module connected to the Arduino Nano.
Scalable Design: Modular components and code structure for easy expansion and modification.

## Hardware Components
* Arduino Nano
* WMYCONGCONG DC 12V 30W High-Speed CW/CCW Permanent Magnet DC Motor (12V DC 3500RPM)
* 37mm Diameter DC Geared Motor
* IR Sensors (x4)
* Bluetooth Module ( for debugging).
* IR Lamp

![image](https://github.com/rperezretana/Tiny-Cheap-Arduino-Lawn-Mower/assets/2858366/d70abb5f-f2c5-4f14-83b4-718e7b83d9e7)

[![Video demo of the lawn mower](https://img.youtube.com/vi/03dL16k8Z_A/0.jpg)](https://www.youtube.com/watch?v=03dL16k8Z_A)

![image](https://github.com/rperezretana/Tiny-Cheap-Arduino-Lawn-Mower/assets/2858366/c8f6a307-5af5-4a3f-8a12-e7fcdb4d9da3)

![image](https://github.com/rperezretana/Tiny-Cheap-Arduino-Lawn-Mower/assets/2858366/d9d967b6-d55b-4eee-a9c9-35b9e8203b1f)

![image](https://github.com/rperezretana/Tiny-Cheap-Arduino-Lawn-Mower/assets/2858366/4247fa73-6522-4642-a8f0-9f3ff464b527)

![image](https://github.com/rperezretana/Tiny-Cheap-Arduino-Lawn-Mower/assets/2858366/8861390a-76cb-4cfe-88a7-ec32593e01dc)

![image](https://github.com/rperezretana/Tiny-Cheap-Arduino-Lawn-Mower/assets/2858366/5b92ccac-73b8-4a3a-b27c-b51e67258a29)


## Project Source Code

This is a recently started project, so it might lack the comments, but when the final version is getting closer I will make
sure to split the files into a more readable version with more comments.
**baseSender**: has the code for the microcontroller that generates a 41khz signal for the boundary wire.
**commandCenter**: it detects the boundary wire and it controls the IR lamp to emit a signal to the mower.
**Core**: this folder has the main code for the mower.

License
This project is licensed under the MIT License - see the LICENSE file for details.

Contact
For any questions or suggestions, please contact:

Rolando Retana
Email: rperezretana@hotmail.com, rperezret@gmail.com
GitHub: github.com/rperezretana
LinkedIn: linkedin.com/in/perezrolando
