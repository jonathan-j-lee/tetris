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
import operator
from geometry_msgs.msg import PoseStamped

from Pieces import *
from tf.transformations import *

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
                    if tile[i][j] == 2: #center of tile
                        centerx, centery = row+i, col+j
                    mat[row+i][col+j] = placementNumber
        return mat, centerx, centery

    def numDigits(self, i):
        numDigits = 0
        while (i != 0):
            i = i // 10
            numDigits += 1
        return numDigits

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
    SQUARETILE = 0
    LINETILE = 1
    STILE = 2
    ZTILE = 3
    REVERSELTILE = 4
    TTILE = 5
    LTILE = 6
    tileTypes = [SquareTile, LineTile, STile, ZTile, ReverseLTile, TTile, LTile]

    r2 = numpy.sqrt(2) / 2
    rotations = [
        (0,     1.0,     0,      0),
        (r2,    r2,     0,      0),
        (1.0,   0.0,     0,      0),
        (r2,    -r2,    0,      0)
    ]
    ROT_0       = 0
    ROT_90      = 1
    ROT_180     = 2
    ROT_270     = 3

    board_ar_marker_id = "ar_marker_8"

    def __init__(self, boardRows=6, boardCols=8, numTiles=[2, 2, 2, 1, 2, 2, 1]):
        self.tiles = self.tileTypes
        self.problem = TetrisSolver(
            boardRows = boardRows,
            boardCols = boardCols,
            tiles = self.tiles,
            numTiles = numTiles
        )        

    def solve(self):
        if self.problem.solveProblem():
            print("Found solution: ")
            
            self.solutionBoard = self.problem.solutionBoard
            printMatrix(self.solutionBoard)

            self.solution = [[], [], [], [], [], [], []]
            for tilePlacement in self.problem.solution:
                tile_index = tilePlacement["tile"]
                row = tilePlacement["row"]
                col = tilePlacement["col"] 
                rotation = tilePlacement["rotation"]
                self.solution[int(tile_index)] += [(row, col, rotation)]
                """
                print("Tile: %s" %(self.tileIndexToType[tile_index]))
                print("\tRow: %i" %(row))
                print("\tColumn: %i" %(col))
                print("\tRotation: %i" %(rotation))
                """
        else:
            print("No solution found")

    def getOrderForPlacement(self):
        """
            Returns array of solution Pieces in order from top left corner of board to bottom right corner
        """
        pieces = []
        maxRow = 0
        for tilePlacement in self.problem.solution:
            tile_index = int(tilePlacement["tile"])
            row = int(tilePlacement["row"])
            col = int(tilePlacement["col"])
            rotation = int(tilePlacement["rotation"])

            pieces += [Piece(tile_index, row, col, rotation)]
            if row > maxRow:
                maxRow = row

        pieces.sort(key=operator.attrgetter('row'))

        #now sort each row by col
        rowsOfPieces = []
        sameRowPieces = []
        currRow = -1
        i = 0
        for piece in pieces:
            if piece.row > currRow: #moved to new row
                currRow = piece.row
                i += 1
                sameRowPieces = []
                rowsOfPieces += [sameRowPieces]
                
            sameRowPieces += [piece]

        pieceOrder = []
        for rowOfPieces in rowsOfPieces:
            rowOfPieces.sort(key=operator.attrgetter('col'))
            pieceOrder.extend(rowOfPieces)
        return pieceOrder

    def getCoordinatesForARTagOfPiece(self, piece):
        """
            Gets coordinates of the Piece (center of AR tag) with respect to top left corner of the board in the board's frame
            Returns the coordinates for the piece as an array of tuples:
                ((x, y, rotation))

            Board top left corner frame:
                ^
              y |
                x -->
        """
        offset_x = Piece.frameWidth / 2
        offset_y = -(Piece.frameWidth / 2)

        #offset x,y should be the exact top left corner of the frame
        x = (piece.col * Piece.tileWidth) + (Piece.tileWidth / 2)
        y = (piece.row * Piece.tileWidth) + (Piece.tileWidth / 2)

        return (x + offset_x, -y + offset_y, piece.rotation)

    def getPoseForPiece(self, piece):
        """
            Returns PoseStamped for Piece (location that the center of mass of the piece should be place) w.r.t. top left of board
        """
        x, y, rot = self.getCoordinatesForARTagOfPiece(piece)
        z = 0
        rotation_quaternion = self.rotations[rot]
        tileType = self.getTileType(piece)

        print("AR tag coordinates: x={}, y={}, rot={}".format(x, y, rot))
        """
            x, y = location for AR tag
            x + comOffset, y + comOffset = location for CoM if not rotated
            rotate by specified rotation about the AR tag point (x, y):
                equivalent to 1) rotate xCoMOffset, yCoMOffset by rot
                2) translate that point by x, y
        """
        rad = (-numpy.pi / 2) * rot #rot = # of rotations by 90 counter CW
        z_axis = (0, 0, 1)
        M = rotation_matrix(rad, z_axis)
        point_CoM = numpy.array([
                [tileType.centerOfMassOffset[0]],
                [tileType.centerOfMassOffset[1]],
                [0],
                [1]
            ])
        rotated_CoM = numpy.dot(M, point_CoM)
        translation = numpy.array([
                [x],
                [y],
                [z],
                [0]
            ])
        transformed_CoM = (rotated_CoM + translation)

        piece_pose = PoseStamped()
        piece_pose.header.frame_id = self.board_ar_marker_id
        piece_pose.pose.position.x = transformed_CoM[0][0]
        piece_pose.pose.position.y = transformed_CoM[1][0]
        piece_pose.pose.position.z = transformed_CoM[2][0] #should be 0
        piece_pose.pose.orientation.x = rotation_quaternion[0]
        piece_pose.pose.orientation.y = rotation_quaternion[1]
        piece_pose.pose.orientation.z = rotation_quaternion[2]
        piece_pose.pose.orientation.w = rotation_quaternion[3]
        print("Piece pose wrt board:")
        print(piece_pose.pose)
        return piece_pose

    def getTileType(self, piece):
        return self.tileTypes[piece.tile_index]

#Helper functions
'''
    Matrices are a list of rows
'''
def printMatrix(mat):
    print("=============================================================================")
    for row in mat:
        for item in row:
            print("\t%s" %(item), end="")
        print("\n")
    print("=============================================================================")
