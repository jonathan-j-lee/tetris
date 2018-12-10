## 1. Introduction

This tetris playing robot plays a game that is slightly different than traditional tetris. While using the same pieces, the robot's goal is to solve a puzzle in which all tetris pieces fill the frame without spaces.

This project demonstrates Baxter's use of computer vision, path planning, sensing, and actuation to complete the task of moving components between two places in a two dimensional plane. We believe that this task is particularly interesting because of the different shaped pieces adding a level of complexity as well as the increasing difficulty to place each piece on the board as the margin of allowed error becomes smaller.

In order to complete this task the robot had to:
  - Identify which tetris piece is placed on the table
  - Find the correct location and orientation of its initial position
  - Pick up the piece with a suction cup correctly with respect to the center of mass
  - Compute the transform from its initial position and orientation to its final orientation
  - Place it into its final location correctly
  
Challenges encountered:
  - The location of the AR tags on the pieces are not very accurate so picking them up and placing them down in the correct location was very difficult
  - Because the suction gripper is not rigid, the pieces have to be picked up at their center of mass in order to keep them horizontal while they are moving to the final location
  - While moving to its desired location, the arm would occasionally knock the piece it was trying to pick up out of the way or hit the piece it was carrying against the table or frame

These skills could be used by robots in construction or in warehouses where the need to identify the pieces correctly as well as move them to the correct location and orientation is great. 

## 2. Design
(a) What design criteria must your project meet? What is the desired functionality?
(b) Describe the design you chose.
(c) What design choices did you make when you formulated your design? What trade-offs did you
have to make?
(d) How do these design choices impact how well the project meets design criteria that would be
encountered in a real engineering application, such as robustness, durability, and efficiency?
## 3. Implementation
(a) Describe any hardware you used or built. Illustrate with pictures and diagrams.
(b) What parts did you use to build your solution?
(c) Describe any software you wrote in detail. Illustrate with diagrams, flow charts, and/or other
appropriate visuals. This includes launch files, URDFs, etc.
(d) How does your complete system work? Describe each step.
## 4. Results
(a) How well did your project work? What tasks did it perform?
(b) Illustrate with pictures and at least one video.
## 5. Conclusion
(a) Discuss your results. How well did your finished solution meet your design criteria?
2
(b) Did you encounter any particular difficulties?
(c) Does your solution have any flaws or hacks? What improvements would you make if you had
additional time?
## 6. Team

James Fang:

Miyuki Weldon: Currently a third-year Mechanical Engineering undergraduate at UC Berkeley with interests in design, dynamics, and robotics.
Her contributions to the project included designing and laser cutting the game pieces and adding the AR tags as well as helping with determining the AR tracking orientations.

Jonathan Lee: Currently a third-year EECS undergraduate at UC Berkeley with interests in robotics, artificial intelligence, and computer systems.
His contributions to the project included integrating AR tracking, writing the project frontend UI, and refining the pick-and-place algorithm.

## 7. Additional materials
(a) code, URDFs, and launch files you wrote
(b) CAD models for any hardware you designed
(c) data sheets for components used in your system
(d) any additional videos, images, or data from your finished solution
(e) links to other public sites (e.g., GitHub), if that is where your files are stored

