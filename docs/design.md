---
title: Design
about: design
---
# Design Criteria 
In order to complete this task the robot had to:
* Identify which tetris piece is placed on the table
* Find the correct location and orientation of its initial position
* Pick up the piece with a suction cup correctly with respect to the center of mass
* Compute the transform from its initial position and orientation to its final orientation
* Place it into its final location correctly


This project will allow us the robot to complete a full game of the tetris puzzle.

# Design Choices
We chose to use computer vision to identify the type of tetris pieces and the board as well as its location and orientation. The suction gripper was chosen to pick up the pieces because of its more stable hold on the thin pieces that the electric gripper did not have. Because the right hand camera was attached to the suction gripper and its poorer quality, the left hand camera was chosen to locate and identify the pieces. The pieces were sanded down to fit more easily into the board. The MoveIt! package was used for path planning and the suction gripper would always initially move to a location that was slightly above where it would pick up and then lower using a sensor to check if the object had been grasped. 
## Trade offs
We originally chose not to use AR tags because the very specified and geometric shapes of the tetris pieces seemed simple to identify, but the increasing simplicity of the AR tags made them more favorable in the end. We chose to put unique AR tags on each type of piece with a specified orientation. We placed each AR tag on the pieces by rastering them into the wood in order to more accurately place their location. The contrast between the dark raster and the light wood was not enough so the AR tags had to be painted. Because each type of piece had only one AR marker, the pieces had to be placed sequentially and individually on the table so the robot would not get confused with multiple of the same AR tag in front of the camera

# Robustness, Durability, and Efficiency of Design
The action of moving the gripper some margin above the object and using the built in sensor to determine whether the suction gripper has grasped the object greatly reduces the negative effects of error in the z direction. Sanding down the edges of the pieces gives the gripper a larger margin of allowed error in the x and y directions by letting the pieces slip more easily into its cut out hole on the board. 

<br/><br/><br/>

