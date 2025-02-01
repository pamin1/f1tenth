## Team 2
Spring 2025 Team 2 GitHub Repository for RACECAR Code.

## Milestones:
- Week 1-2: Basics
  - Car following algorithm
  - New member starter project
- Week 3-6: Mapping
  - Choose some mapping package
  - Get sim working with RVIZ, visualize track data
  - Map a track
- Week 7-9: Localization
  - With slow moving car, show localized position of the car
- Week 10-11: Planning and Control
  - Trajector optimization on occupancy grid
  - Pure Pursuit, Stanley

## Current Progress:
**Week 1-2**

Prachit:
* Working on a following controller in simulation. We simplify the problem by assuming the position of the opponent car by subscribing to the /opp_odom topic. 
* Using the known global position, we localize the frame of reference and use PID control to maneuver the ego agent towards the opponent.
* Removed global position abstractions by narrowing the line of data we use from the lidar. Currently at a naive lidar detection implementation which averages the
  the middle ~50 laser scans. Using Follow The Gap for the underlying controller.

Vaishnavi:
* Set up a VM, played around with ROS2 (familiar with writing a publisher, subcriber, service and client in C++ and Python)
* Testing out composing ROS2 nodes that can read data from a sensor and alter the servo motor based on the readings (in progress)
* Will test the above during Week 2 on physical car
