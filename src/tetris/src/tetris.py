#!/usr/bin/env python

"""
tetris -- Solves a Tetris-like puzzle.
"""

import rospy

from pnp import TetrisPNPTask
from solver import (TetrisSolver, SquareTile, LTile, ReverseLTile, TTile,
                    LineTile, ZTile, STile)


# TODO: add face


def solve_puzzle():
    rows = int(rospy.get_param('board_height'))
    columns = int(rospy.get_param('board_width'))
    tile_params = ['board_sqtiles', 'board_linetiles', 'board_stiles',
                   'board_ztiles', 'board_revltiles', 'board_ttiles',
                   'board_ttiles']
    solver = TetrisSolver(rows, columns,
        [SquareTile, LineTile, STile, ZTile, ReverseLTile, TTile, LTile],
        [int(rospy.get_param(param)) for param in tile_params])
    if not solver.solveProblem():
        raise ValueError('Failed to solve Tetris board.')
    return solver.solutionBoard


def add_table_obstacle(task, margin=0.01):
    task.planner.add_box_obstacle([1, 1, 0.5], 'table', )


def main():
    rospy.init_node('tetris')
    verbose = rospy.get_param('verbose').lower() == 'true'
    task = TetrisPNPTask(verbose=verbose)
    solution = solve_puzzle()

    while not rospy.is_shutdown():
        raw_input('Press enter.')

    # solver = TetrisSolver(board_rows=6, board_cols=8,
    # tiles=[SquareTile, LTile, ReverseLTile, TTile, LineTile, ZTile, STile], num_tiles=[2, 2, 2, 1, 2, 2, 1])


if __name__ == '__main__':
    main()
