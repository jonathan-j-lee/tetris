"""
This "Tetris" solver finds a solution to a given tetris tiling problem. It
uses a brute force solver. Additional tiles can be added by adding more
classes that inherit from Tile.
"""

from __future__ import print_function

__author__ = "Caleb Begly"
__copyright__ = "Copyright 2017 Caleb Begly"
__license__ = "MIT"
__maintainer__ = "Caleb Begly"

import copy
import numpy


class TetrisSolver:
    """ Sets up a new tiling problem solver. """
    def __init__(self, board_rows, board_cols, tiles, num_tiles):
        self.board_rows = board_rows
        self.board_cols = board_cols
        self.num_tiles = num_tiles
        self.solution = []
        self.solution_board = None

        # Populate rotations and tiles
        self.unique_rotations = []
        self.tiles = []
        for tile in tiles:
            self.tiles.append(tile.tile)
            self.unique_rotations.append(tile.unique_rotations)

    def solve_problem(self):
        """
        This takes a problem and attempts to find a solution.
        Returns true if a solution exists, false otherwise.
        """
        board = numpy.zeros((self.board_rows, self.board_cols), dtype=numpy.int)
        return self.has_solution(board, self.tiles, self.num_tiles)

    def generate_board(self, rows, columns):
        board = []
        for _ in range(rows):
            board.append(list(range(columns)))
        return board

    def place_tile(self, matrix, tile, row, col, placement_number):
        """ Places tile on matrix (zero-indexed). """
        matrix = copy.deepcopy(matrix)
        rows = len(matrix)
        columns = len(matrix[0])
        tile_rows = len(tile)
        tile_columns = len(tile[0])
        if tile_rows + row > rows or row < 0:
            raise ValueError("Row is out of bounds")
        if tile_columns + col > columns or col < 0:
            raise ValueError("Col is out of bounds")

        center_x, center_y = row, col
        for i in range(tile_rows):
            for j in range(tile_columns):
                if tile[i][j] > 0: #solid piece of tile, place it
                    suffix = "0" * self.num_digits(placement_number)
                    if tile[i][j] == 2:
                        center_x, center_y = row + i, col + j
                        suffix = str(placement_number)
                    matrix[row+i][col+j] = int(str(placement_number) + suffix)
        return matrix, center_x, center_y

    def num_digits(self, i):
        num_digits = 0
        while i != 0:
            i = i // 10
            num_digits += 1
        return num_digits

    def can_place_tile(self, matrix, tile, row, col):
        """ Checks if we can place the tile at the given location. """
        rows = len(matrix)
        columns = len(matrix[0])
        tile_rows = len(tile)
        tile_columns = len(tile[0])
        if tile_rows + row > rows or row < 0:
            return False # Row out of bounds on one end or the other
        if tile_columns + col > columns or col < 0:
            return False # Col out of bounds on one end or the other

        # Confirm that, in each place in the tile where there is a non-zero
        # entry, the matrix has a space open (a zero entry)
        for i in range(tile_rows):
            for j in range(tile_columns):
                if tile[i][j] and matrix[row+i][col+j]:
                    return False
        return True

    def has_solution(self, board, tiles, num_tiles, row_start=0, col_start=0, placement=1):
        """ Returns true if a solution exists for the given parameters. """
        rows, columns = len(board), len(board[0])

        # Note, the i, j loops just move to the next empty tile. Since the tile
        # shapes are complex, this is a decent way of doing it.
        for i in range(row_start, rows):
            inner_start = 0
            if i == row_start:  # For the first row, ignore columns < col_start.
                inner_start = col_start
            for j in range(inner_start, columns):
                if not board[i][j]:
                    #Attempt to insert each tile
                    for tile_num, tile in enumerate(tiles):
                        if num_tiles[tile_num]: # If there are more available tiles of this kind to place
                            for rot in range(self.unique_rotations[tile_num]):
                                col_offset = self.find_offset(tile)
                                if self.can_place_tile(board, tile, i, j + col_offset): # If it works, see if we can solve using the new solution
                                    board2, center_r, center_c = self.place_tile(board, tile, i, j + col_offset, placement)
                                    num_tiles2 = copy.deepcopy(num_tiles)
                                    # We placed the tile, so it is no longer available.
                                    num_tiles2[tile_num] -= 1
                                    if self.has_solution(board2, tiles, num_tiles2, i, j, placement + 1):
                                        #Store this part of the solution
                                        self.solution.append({
                                            "row": center_r,
                                            "col": center_c,
                                            "tile": tile_num,
                                            "rotation": rot
                                        })
                                        return True
                                #Rotate tile for next try
                                tile = Tile.rotate_tile(tile)
                    # There is no tile, or rotated tile, that can fill the cell,
                    # so this path can't be part of a solution.
                    return False

        if self.is_full_solution(board):
            self.solution_board = board
            return True
        return False

    def is_full_solution(self, board):
        """ Check for a solved board. """
        for i, row in enumerate(board):
            for j in range(len(row)):
                if not board[i][j]:
                    return False
        return True

    def find_offset(self, tile):
        """
        Computes the column offset needed so there will be a tile at the
        current location (only horizontal offset). Vertical offset it not
        allowed because the solver invarient requires all items to the left and
        above have to be.
        """
        for j in range(len(tile[0])):
            if tile[0][j] > 0:
                return -j


class Piece:
    def __init__(self, tile_index, row, col, rotation):
        self.tile_index = tile_index
        self.row = row
        self.col = col
        self.rotation = rotation


class Tile:
    """ The base class for any tiles used. """
    # By default, there are 4 unique rotational positions.
    unique_rotations = 4

    @staticmethod
    def rotate_tile(tile):
        """ Rotate the tile 90 degrees clockwise. """
        tile_rows = len(tile)
        tile_columns = len(tile[0])
        # After the rotation, the new matrix has number of rows and columns switched.
        rotated_tile = numpy.zeros((tile_columns, tile_rows), dtype=numpy.int)

        # Copy the values over
        for i in range(tile_rows):
            for j in range(tile_columns):
                rotated_tile[j][tile_rows - i - 1] = tile[i][j]
        return rotated_tile


class SquareTile(Tile):
    tile = [
        [1, 2],
        [1, 1],
    ]
    ar_tag_num = 0
    unique_rotations = 1


class LTile(Tile):
    tile = [
        [2, 1],
        [0, 1],
        [0, 1],
    ]
    ar_tag_num = 6


class ReverseLTile(Tile):
    tile = [
        [2, 0, 0],
        [1, 1, 1],
    ]
    ar_tag_num = 4


class TTile(Tile):
    tile = [
        [1, 1, 1],
        [0, 2, 0],
    ]
    ar_tag_num = 5


class LineTile(Tile):
    tile = [
        [2],
        [1],
        [1],
        [1],
    ]
    ar_tag_num = 1
    unique_rotations = 2


class ZTile(Tile):
    tile = [
        [0, 1],
        [1, 2],
        [1, 0],
    ]
    ar_tag_num = 3
    unique_rotations = 2


class STile(Tile):
    tile = [
        [1, 0],
        [1, 2],
        [0, 1],
    ]
    ar_tag_num = 2
    unique_rotations = 2
