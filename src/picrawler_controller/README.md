# Picrawler controller package
This package contains all the nodes required to control the picrawler:
- Digital pin node
- Robot hat node
- ADC node
- IIC node
- Servo contols
- Walking controller

File sctructure:
```
picrawler_controller
│
├── picrawler_digital_pin
│   │
│   ├── /pinX/read -> read from digital pin
│   └── /pinX/write -> write to digital pin
│
├── picrawler_robot_hat
│   │
│   ├── /robot_hat/reset -> resets the MCU (service)
│   ├── /robot_hat/led -> Sets the onboard LED (Bool)
│   ├── /robot_hat/buttons -> Publishes the button's states (2 Bool's)
│   └── /robot_hat/battery -> Publishes the battery percent (float)
│
├── picrawler_servo_control
│   │
│   ├── /legX/target_pose -> Set a new target position
│   └── /legX/pose -> Current position of the leg
```

### Robot Hat Node
This node is responsible for all the misc or debugging functions on the robot hat itself. 
- Onboard LED
- Onboard buttons
- MCU reset
- Pattery percentage

### Servo controlls
This is primarily an interal use node that moves each leg based on an inverse kinematics model.