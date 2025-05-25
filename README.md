# Voyegers
This multitasking robot was designed to complete several tasks: line following, wall following, and pick & place operations.
This is the first robot we ever built , a team project developed as part of our Embedded Systems module.

Key features:
 • Line Detection: An QTR-8A IR sensor array was used for line following, with 2 additional IR sensors for junction detection (total of 10 IR sensors).
 • Microcontroller: ESP32
 • Multiplexing: CD4051 MUX was used to reduce the number of required GPIO pins.
 • Box Detection: A front-facing IR sensor triggered an interrupt in the program.
 • Wall Following: Combined input from a front-mounted IR sensor and a side-mounted TOF sensor.
 • Drive System: Two 6V 300 RPM N20 motors, powered at 12V using a TB6612 motor driver.
 • Robotic Arm: Custom-designed 2-DOF arm, 3D printed and driven by two SG90 servo motors.
 • Power Supply: 11.1V 1500mAh LiPo battery with multiple buck converters.
 • Assembly: All components were soldered on a double-sided dot board and mounted onto an acrylic base.

![alt text](https://github.com/BinethGeesara/Voyegers/blob/4440dd22125c8ca9071cd6539414d0aa0c330160/IMG_7569.jpg)
