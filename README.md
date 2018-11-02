# Tetris Player

## Plan
- Develop a piece-recognition node.
	- This node will detect the shapes of a given set of Tetris pieces placed in a starting area using Baxter’s cameras.
- Develop a Tetris solver.
	- Given a set of Tetris pieces, the solver will return a complete placement for them in the form of an array representing the board. This will be our solution configuration.
- Develop a piece-placing node.
	- This node will use Baxter’s suction end effector to place the pieces in the solution configuration using closed-loop control.