---
about: implementation
---

(a) Describe any hardware you used or built. Illustrate with pictures and diagrams. (b) What parts did you use to build your solution? (c) Describe any software you wrote in detail. Illustrate with diagrams, flow charts, and/or other appropriate visuals. This includes launch files, URDFs, etc. (d) How does your complete system work? Describe each step.

We used the laser cutters (makerspace at Jacobs Institute for Design Innovation) to cut our own Tetris pieces and a board to place them from 1/4-inch thick plywood. The Tetris pieces are so-called because each piece is some union of 4 square blocks. Each square block is 2.5 x 2.5 inches (~6 x 6 cm) and the board can fit 6 x 8 of these blocks. We refer to the 7 unique pieces as the Square, Line, S, Z, Reverse L, T, and L pieces. 

![Pieces with AR + CoM]({{ site.baseurl }}/assets/images/completed-puzzle.png) <!--change this-->

The picture above shows each piece and which AR marker is rasterized on it and where (black dot), as well as where we tell our pick-and-place code to pick up the piece (red dot) using a suction gripper. We want this red dot to be as close to the center of mass of the piece as possible, which we can generally locate using the symmetry of the pieces. This is desirable for stability of the piece during pick-and-place, allowing for easier placement (it's a tight fit on the board). However, since the rasterized AR marker is an uneven surface, it was hard for the suction gripper to get a good seal if it was overlapping the AR marker. Therefore, we had to place the red dots such that the suction gripper (diameter 3.5 cm) would not overlap with the AR marker when it grips the piece while still gripping close enough to the piece's center of mass.

To complete its task, we have the Baxter robot:
1) __Figure out which Tetris pieces it's working with and compute a Tetris solution__. 
We input the 7 types of pieces along with how many of each we're using and the size of the board (in block units) into a Tetris solver which outputs a complete board solution. 

We define each piece as an instance of a Tile object. We specify its shape using a matrix of 0's and 1's. Each piece also contains information about its AR marker id, its name, the unique rotations for the piece, and the offset from the center of the AR marker on the piece to the red dot (as drawn above) in meters. The solver itself uses an algorithm (designed by Caleb Begly of MIT) which iterates over each piece and its possible rotations to find a feasible way to place it on the board (a matrix) such that there are no gaps.
2) __From the solution, choose a piece to execute pick-and-place__.
We have the solver output the locations of each piece in the solved puzzle in row-major order. We also want pick-and-place to execute in row major order so that we can more easily enforce a tight fit on the board (i.e. rather than first placing a piece in the middle of the board, instead place a piece that belongs in one of the corners and align it with the board frame). 

Therefore, we simply choose the next piece in row major order as given by the solution. The code will request the user for the selected piece type. The user will place the requested piece so that the left-hand camera can see it and the right hand can grip it.
3) __Pick up the chosen piece at its red dot__.
We first have Baxter locate the piece's AR marker. From the dimensions of the piece and the red dot offsets, we can compute the location of the red dot in the AR marker's frame. Note that since the AR marker's frame also specifies an orientation with respect to the base, the red dot coordinates also include an orientation. This means that the gripper will always pick up a given piece in the same orientation and position.

We then use `tf2` to transform the red dot coordinates to Baxter's base frame and direct the right gripper to go to that pose (position + orientation) and activate the vacuum gripper. 

To ensure that we have picked up the piece, we 
4) move to it’s solution place on board and drop
	i) rotate to match board orientation
	ii) rotate to piece’s rotation w.r.t. board
