'''
    Example usage of Tetris Solver
'''

from TetrisSolver import *

print("====================Our Puzzle=====================")
"""
[LineTile, TTile, SquareTile, ReverseLTile, LTile, STile, ZTile]
"""
solver = OurSolver(
    boardRows = 6,
    boardCols = 8,
    numTiles = [2, 2, 2, 2, 1, 2, 1]
)
solver.solve()
#print(solver.getCoordinatesForARTagOfPiece(OurSolver.SQUARETILE))
for piece in solver.getOrderForPlacement():
    print("***************Piece: {}".format(solver.tileIndexToType[piece.tile_index]))
    print("\tR: {}, C: {}, rot: {}".format(piece.row, piece.col, piece.rotation))
    print("\tARTagCoordinates: {}".format(solver.getCoordinatesForARTagOfPiece(piece)))
    print("\tCoMPose: {}".format(solver.getPoseForPiece(piece).pose))

"""
#Example 1: Full example with error handling and table output
print("====================Example 1=====================")
problem = TetrisSolver(
    boardRows = 6,
    boardCols = 6,
    tiles = [LTile, SquareTile, STile],
    numTiles = [4, 1, 4]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
    for tilePlacement in problem.solution:
        print("Tile: %i" %(tilePlacement["tile"]))
        print("\tRow: %i" %(tilePlacement["row"]))
        print("\tColumn: %i" %(tilePlacement["col"]))
        print("\tRotation: %i" %(tilePlacement["rotation"]))
else:
    print("No solution found")

#Example 2: Custom solution output handling
customTiles = [LTile, SquareTile];
print("====================Example 2=====================")
problem = TetrisSolver(
    boardRows = 4,
    boardCols = 6,
    tiles = customTiles,
    numTiles = [4, 2]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
    for tilePlacement in problem.solution:
        print("Tile: %i" %(tilePlacement["tile"]))
        print("\tRow: %i" %(tilePlacement["row"]))
        print("\tColumn: %i" %(tilePlacement["col"]))
        print("\tRotation: %i" %(tilePlacement["rotation"]))
else:
    print("No solution found")

#Examples 3 and up: Additional example puzzles
print("====================Example 3=====================")
problem = TetrisSolver(
    boardRows = 6,
    boardCols = 8,
    tiles = [LTile, ReverseLTile, TTile, LineTile, SquareTile, ZTile],
    numTiles = [2, 3, 2, 1, 2, 2]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
    for tilePlacement in problem.solution:
        print("Tile: %i" %(tilePlacement["tile"]))
        print("\tRow: %i" %(tilePlacement["row"]))
        print("\tColumn: %i" %(tilePlacement["col"]))
        print("\tRotation: %i" %(tilePlacement["rotation"]))
else:
    print("No solution found")


print("====================Example 4=====================")
problem = TetrisSolver(
    boardRows = 6,
    boardCols = 6,
    tiles = [LTile, ReverseLTile, LineTile, SquareTile, ZTile],
    numTiles = [2, 2, 1, 2, 2]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")


print("====================Example 5=====================")
problem = TetrisSolver(
    boardRows = 10,
    boardCols = 4,
    tiles = [LTile, TTile, LineTile, SquareTile, STile],
    numTiles = [2, 4, 1, 2, 1]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")


print("====================Example 6=====================")
problem = TetrisSolver(
    boardRows = 6,
    boardCols = 6,
    tiles = [LTile, ReverseLTile, TTile, ZTile],
    numTiles = [2, 2, 2, 3]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")


print("====================Example 7=====================")
problem = TetrisSolver(
    boardRows = 6,
    boardCols = 6,
    tiles = [LTile, ReverseLTile, TTile, LineTile, SquareTile, STile],
    numTiles = [2, 1, 2, 1, 2, 1]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")


print("====================Example 8=====================")
problem = TetrisSolver(
    boardRows = 8,
    boardCols = 5,
    tiles = [LTile, TTile, LineTile, SquareTile, ZTile, STile],
    numTiles = [2, 2, 2, 1, 2, 1]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")


print("====================Example 9=====================")
problem = TetrisSolver(
    boardRows = 8,
    boardCols = 6,
    tiles = [LTile, ReverseLTile, TTile, SquareTile, ZTile, STile],
    numTiles = [1, 1, 4, 2, 2, 2]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")

print("====================Example 10=====================")
problem = TetrisSolver(
    boardRows = 7,
    boardCols = 8,
    tiles = [LTile, ReverseLTile, TTile, LineTile, SquareTile, ZTile, STile],
    numTiles = [1, 1, 4, 2, 4, 1, 1]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")


print("====================Example 11=====================")
problem = TetrisSolver(
    boardRows = 5,
    boardCols = 8,
    tiles = [LTile, ReverseLTile, TTile, LineTile, SquareTile],
    numTiles = [2, 1, 4, 2, 1]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")


print("====================Example 12=====================")
problem = TetrisSolver(
    boardRows = 6,
    boardCols = 6,
    tiles = [LTile, SquareTile, STile],
    numTiles = [4, 1, 4]
)
if problem.solveProblem():
    print("Found solution: ")
    printMatrix(problem.solutionBoard)
else:
    print("No solution found")
"""