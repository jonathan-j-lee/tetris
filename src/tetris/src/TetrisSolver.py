'''
    This "Tetris" solver finds a solution to a given tetris tiling problem. It
    uses a brute force solver. Additional tiles can be added by adding more
    classes that inherit from Tile.
'''
from __future__ import print_function

__author__ = "Caleb Begly"
__copyright__ = "Copyright 2017 Caleb Begly"
__license__ = "MIT"
__maintainer__ = "Caleb Begly"

import numpy
import copy

class TetrisSolver(object):
    '''
        Sets up a new tiling problem solver
    '''
    def __init__(self, boardRows, boardCols, tiles, numTiles):
        self.boardRows = boardRows
        self.boardCols = boardCols
        self.numTiles = numTiles
        self.solution = []
        self.solutionBoard = None

        #Populate rotations and tiles
        self.uniqueRotations = []
        self.tiles = []
        for tile in tiles:
            self.tiles.append(tile.tile)
            self.uniqueRotations.append(tile.uniqueRotations)

    '''
        This takes a problem and attempts to find a solution.
        Returns true if a solution exists, false otherwise
    '''
    def solveProblem(self):
        board = numpy.zeros((self.boardRows, self.boardCols), dtype=numpy.int)
        return self.hasSolution(board, self.tiles, self.numTiles)

    '''
        Generate game board with certain number of rows and cols
    '''
    def genBoard(self, rows, cols):
        board = []
        for i in range(rows):
            board.append(range(cols))
        return board

    '''
        Places tile on matrix (row and col are zero based)
    '''
    def placeTile(self, mat, tile, row, col, placementNumber):
        mat = copy.deepcopy(mat)
        rows = len(mat)
        cols = len(mat[0])
        tileRows = len(tile)
        tileCols = len(tile[0])
        if(tileRows + row > rows or row < 0):
            raise ValueError("Row is out of bounds")
        if(tileCols + col > cols or col < 0):
            raise ValueError("Col is out of bounds")

        centerx, centery = row, col
        for i in range(tileRows):
            for j in range(tileCols):
                if tile[i][j] > 0: #solid piece of tile, place it
                    mat[row+i][col+j] += placementNumber
                    if tile[i][j] == 2: #center of tile
                        centerx, centery = row+i, col+j
        return mat, centerx, centery

    '''
        Checks if we can place the tile at the given location.
    '''
    def canPlaceTile(self, mat, tile, row, col):
        rows = len(mat)
        cols = len(mat[0])
        tileRows = len(tile)
        tileCols = len(tile[0])
        if(tileRows + row > rows or row < 0):
            return False #Row out of bounds on one end or the other
        if(tileCols + col > cols or col < 0):
            return False #Col out of bounds on one end or the other

        #Confirm that, in each place in the tile where there is a non-zero entry, the matrix has a space open (a zero entry)
        for i in range(tileRows):
            for j in range(tileCols):
                if(tile[i][j] > 0 and mat[row+i][col+j] > 0):
                    return False
        return True;

    '''
        Returns true if a solution exists for the given parameters
    '''
    def hasSolution(self, board, tiles, numTiles, rowStart = 0, colStart = 0, placement = 1):
        rows = len(board)
        cols = len(board[0])

        #Note, the i, j loops just move to the next empty tile. Since the tile shapes are complex, this is a decent way of doing it.
        for i in range(rowStart, rows): #For each row
            innerStart = 0
            if i == rowStart: #For the first row, ignore columns < colStart
                innerStart = colStart
            for j in range(innerStart, cols): #For each column
                if board[i][j] == 0: #If the cell is empty
                    #Attempt to insert each tile
                    for tileNumber in range(len(tiles)): #Loop through all the tiles
                        if numTiles[tileNumber] > 0: #If there are more available tiles of this kind to place
                            tile = tiles[tileNumber] #Get the tile we will use
                            for rot in range(self.uniqueRotations[tileNumber]):
                                colOffset = self.findOffset(tile)
                                if self.canPlaceTile(board, tile, i, j + colOffset): #If it works, see if we can solve using the new solution
                                    board2, center_r, center_c = self.placeTile(board, tile, i, j + colOffset, placement)
                                    numTiles2 = copy.deepcopy(numTiles)
                                    numTiles2[tileNumber] -= 1 #We placed the tile, so it is no longer available
                                    if self.hasSolution(board2, tiles, numTiles2, i, j, placement + 1):
                                        #Store this part of the solution
                                        self.solution.append({
                                            "row": center_r,
                                            "col": center_c,
                                            "tile": tileNumber,
                                            "rotation": rot
                                        })
                                        return True
                                #Rotate tile for next try
                                tile = Tile.rotateTile(tile)
                    return False #There is no tile, or rotated tile, that can fill the cell, so this path can't be part of a solution.

        #At this point we are done. We either have a solution, or we don't
        if self.isFullSolution(board):
            self.solutionBoard = board
            return True
        else:
            return False

    '''
        Check for a solved board
    '''
    def isFullSolution(self, board):
        rows = len(board)
        cols = len(board[0])

        #Confirm that every place has a nonzero entry
        for i in range(rows):
            for j in range(cols):
                if board[i][j] == 0:
                    return False
        return True

    '''
        Computes the column offset needed so there will be a tile at the current location (only horizontal offset)
        Vertical offset it not allowed because the solver invarient requires all items to the left and above have to be
    '''
    def findOffset(self, tile):
        for j in range(len(tile[0])):
            if tile[0][j] > 0:
                return -j

class OurSolver(object):

    """
        # Pieces to AR Tags 
        {
            'SquareTile':0,
            'LineTile':1,
            'STile':2,
            'ZTile':3, 1
            'ReverseLTile':4,
            'TTile':5,
            'LTile':6, 1
            'FrameCorner':7,
        }
    """
    def __init__(self, boardRows=6, boardCols=8, numTiles=[2, 2, 2, 1, 2, 2, 1]):
        self.tiles = [SquareTile, LineTile, STile, ZTile, ReverseLTile, TTile, LTile]
        self.problem = TetrisSolver(
            boardRows = boardRows,
            boardCols = boardCols,
            tiles = self.tiles,
            numTiles = numTiles
        )

        self.SQUARETILE = 0
        self.LINETILE = 1
        self.STILE = 2
        self.ZTILE = 3
        self.REVERSELTILE = 4
        self.TTILE = 5
        self.LTILE = 6

    def solve(self):
        if self.problem.solveProblem():
            print("Found solution: ")
            
            self.solutionBoard = self.problem.solutionBoard
            printMatrix(self.solutionBoard)

            self.solution = [[], [], [], [], [], [], []]
            for tilePlacement in self.problem.solution:
                tileNumber = tilePlacement["tile"]
                row = tilePlacement["row"]
                col = tilePlacement["col"] 
                rotation = tilePlacement["rotation"]
                print("Tile: %i" %(tileNumber))
                print("\tRow: %i" %(row))
                print("\tColumn: %i" %(col))
                print("\tRotation: %i" %(rotation))
                self.solution[int(tileNumber)] += [(row, col, rotation)]
        else:
            print("No solution found")

    def getCoordinatesForPiece(self, piece):
        """
            piece = index of piece (e.g. self.LINETILE)

            Gets coordinates of all pieces with respect to top left corner

            TODO: figure out orientation
        """
        for piecePlacement in self.solution[piece]:


'''
	The base class for any tiles used.
'''
class Tile:
    tile = [[]]
    uniqueRotations = 4 #By default, there are 4 unique rotational positions.

    '''
        Rotate the tile 90 degrees CW
    '''
    @staticmethod
    def rotateTile(tile):
        tileRows = len(tile)
        tileCols = len(tile[0])
        rotTile = numpy.zeros((tileCols, tileRows), dtype=numpy.int) #After the rotation, the new matrix has number of rows and columns switched.

        #Copy the values over
        for i in range(tileRows):
            for j in range(tileCols):
                rotTile[j][tileRows - i - 1] = tile[i][j]
        return rotTile

'''
    Square Tile
'''
class SquareTile(Tile):
    tile = [
        [1, 2],
        [1, 1]
    ]
    uniqueRotations = 1

'''
    Left L Tile
'''
class LTile(Tile):
    tile = [
        [1, 1, 1],
        [2, 0, 0]
    ]

'''
    Right L Tile
'''
class ReverseLTile(Tile):
    tile = [
        [2, 0, 0],
        [1, 1, 1]
    ]

'''
    T Tile
'''
class TTile(Tile):
    tile = [
        [1,1,1],
        [0,2,0]
    ]

'''
    Line Tile
'''
class LineTile(Tile):
    tile = [
        [2,1,1,1]
    ]
    uniqueRotations = 2

'''
    Z Tile
'''
class ZTile(Tile):
    tile = [
        [1,2,0],
        [0,1,1]
    ]
    uniqueRotations = 2

'''
    S Tile
'''
class STile(Tile):
    tile = [
        [0,2,1],
        [1,1,0]
    ]
    uniqueRotations = 2

#Helper functions
'''
    Matrices are a list of rows
'''
def printMatrix(mat):
    print("===========================================================")
    for row in mat:
        for item in row:
            print("\t%s" %(item), end="")
        print("\n")
    print("===========================================================")
