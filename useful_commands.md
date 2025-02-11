# Hand control:

### Home all

`. install/setup.bash`  
`ros2 launch hand_servo_control multi_servo.launch.xml usb:=/dev/ttyUSB1`

on another terminal  

`ros2 service call /set_servo_positions hand_servo_interfaces/srv/ServoCommands "{channels: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15], positions: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]}"`

### Servo motion test

`ros2 service call /set_servo_positions hand_servo_interfaces/srv/ServoCommands "{channels: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15], positions: [-90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90, -90]}" `


`ros2 service call /set_servo_positions hand_servo_interfaces/srv/ServoCommands "{channels: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15], positions: [90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90, 90]}" `

### Shadowing

    ros2 launch hand_motion_shadowing shadowing.launch.xml usb:=/dev/ttyUSB0s

