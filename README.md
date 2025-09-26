# ROS2 Learning Project

This project is a ROS2 learning exercise built around the classic package.  
The goal is to practice core ROS2 concepts such as nodes, topics, services, and launch files, through drawing fun mathematical shapes.

---

## Features
- Draws different shapes using parametric equations:
  -  Star
  -  Leaf
  -  Flower 
  -  Butterfly 
- Supports commands:
  - clear → clears the screen and re-centers the turtle
  - stop → stops the turtle mid-drawing
- Shapes are centered and scaled to fit inside the turtlesim window.
- Two drawing approaches tested:
  - **Velocity-based drawing** (`/cmd_vel`): moves turtle step by step. Fun but not very precise for complex parametric curves.
  - **Teleportation-based drawing** (`/teleport_absolute`): directly places turtle at parametric (x, y) positions, producing much more accurate shapes.

## Topics and Nodes
<img width="1176" height="97" alt="image" src="https://github.com/user-attachments/assets/a3010f95-d33a-4c34-815d-fd5e0018cca1" />
