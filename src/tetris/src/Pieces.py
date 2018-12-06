import numpy as np

class Piece:
    def __init__(self, tile_index, row, col, rotation):
        self.tile_index = tile_index
        self.row = row
        self.col = col
        self.rotation = rotation

'''
	The base class for any tiles used.
'''
class Tile:
    tile = [[]]
    uniqueRotations = 4 #By default, there are 4 unique rotational positions.

    inchToM = .0254 #1" = 2.54cm
    tileWidth = 2.5 * inchToM #.0635

    '''
        Rotate the tile 90 degrees CW
    '''
    @staticmethod
    def rotateTile(tile):
        tileRows = len(tile)
        tileCols = len(tile[0])
        rotTile = np.zeros((tileCols, tileRows), dtype=np.int) #After the rotation, the new matrix has number of rows and columns switched.

        #Copy the values over
        for i in range(tileRows):
            for j in range(tileCols):
                rotTile[j][tileRows - i - 1] = tile[i][j]
        return rotTile

"""
    0: empty space
    1: tile
    2: tile with AR tag
    centerOfMassOffset: offset from AR tag center

        ^
      y |
        x -->
"""

'''
    Square Tile
    AR Tag 0
'''
class SquareTile(Tile):
    tile = [
        [1, 2],
        [1, 1]
    ]
    uniqueRotations = 1

    ar_marker_id = "ar_marker_0"
    xOffset = (np.sqrt(2) / 100) + (Tile.tileWidth / 2)
    yOffset = - ((np.sqrt(2) / 100) + (Tile.tileWidth / 2))
    centerOfMassOffset = (xOffset, yOffset)

'''
    Line Tile
    AR Tag 1
'''
class LineTile(Tile):
    #[2,1,1,1]
    tile = [
        [2],
        [1],
        [1],
        [1]
    ]
    uniqueRotations = 2

    ar_marker_id = "ar_marker_1"
    xOffset = 0
    yOffset = -(Tile.tileWidth * 3 / 2)
    centerOfMassOffset = (xOffset, yOffset)

'''
    S Tile
    AR Tag 2
'''
class STile(Tile):
    '''[0,2,1],
        [1,1,0]'''
    tile = [
        [1,0],
        [1,2],
        [0,1]
    ]
    uniqueRotations = 2

    ar_marker_id = "ar_marker_2"
    xOffset = -((Tile.tileWidth / 2) + (2 / 100))
    yOffset = 0
    centerOfMassOffset = (xOffset, yOffset)

'''
    Z Tile
    AR Tag 3
'''
class ZTile(Tile):
    '''[1,2,0],
        [0,1,1]'''
    tile = [
        [0,1],
        [2,1],
        [1,0]
    ]
    uniqueRotations = 2

    ar_marker_id = "ar_marker_3"
    xOffset = (Tile.tileWidth / 2) + (2/100)
    yOffset = 0
    centerOfMassOffset = (xOffset, yOffset)

'''
    Right L Tile
    AR Tag 4
'''
class ReverseLTile(Tile):
    tile = [
        [2, 0, 0],
        [1, 1, 1]
    ]

    ar_marker_id = "ar_marker_4"
    xOffset = (Tile.tileWidth / 2) + (1.75/100)
    yOffset = - (Tile.tileWidth)
    centerOfMassOffset = (xOffset, yOffset)

'''
    T Tile
    AR Tag 5
'''
class TTile(Tile):
    tile = [
        [1,1,1],
        [0,2,0]
    ]

    ar_marker_id = "ar_marker_5"
    xOffset = 0
    yOffset = (Tile.tileWidth / 2) + (2/100) 
    centerOfMassOffset = (xOffset, yOffset)

'''
    Left L Tile
    AR Tag 6
'''
class LTile(Tile):
    '''[2, 0, 0],
        [1, 1, 1]'''
    tile = [
        [2, 1],
        [0, 1],
        [0, 1]
    ]

    ar_marker_id = "ar_marker_6"
    xOffset = Tile.tileWidth
    yOffset = - ((Tile.tileWidth / 2) + (1.75 / 100))
    centerOfMassOffset = (xOffset, yOffset)