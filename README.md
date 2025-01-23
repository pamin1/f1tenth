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
* Further improvements will include PID tuning and implementation of RRT* to do real time path planning.
