---
title: Design
about: design
---

![Completed puzzle]({{ site.baseurl }}/assets/images/completed-puzzle.png)

# Design Criteria 
In order to complete this task the robot had to:
    - Identify which tetris piece is placed on the table
    - Find the correct location and orientation of its initial position
    - Pick up the piece with a suction cup correctly with respect to the center of mass
    - Compute the transform from its initial position and orientation to its final orientation
    - Place it into its final location correctly

This project will allow us the robot to complete a full game of the tetris puzzle.

# Design Choices
We chose to use computer vision to identify the type of tetris pieces and the board as well as its location and orientation. The suction gripper was chosen to pick up the pieces because of its more stable hold on the thin pieces that the electric gripper did not have. Because the right hand camera was attached to the suction gripper and its poorer quality, the left hand camera was chosen to locate and identify the pieces.
## Trade offs
We originally chose not to use AR tags because the very specified and geometric shapes of the tetris pieces seemed simple to identify, but the increasing simplicity of the AR tags made them more favorable in the end. We chose to put unique AR tags on each type of piece with a specified orientation. We placed each AR tag on the pieces by rastering them into the wood in order to more accurately place their location. The contrast between the dark raster and the light wood was not enough so the AR tags had to be painted.
