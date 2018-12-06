#!/usr/bin/env python

"""
tetris -- Solves a Tetris-like puzzle.
"""

from __future__ import division, generators, print_function, unicode_literals
import rospy
from pnp import TetrisPNPTask
from solver import TILE_TYPES, solve_puzzle, optimize_solution, display_solution


# TODO: add face


def get_board_state():
    """
    Returns (int, int, dict): Rows, columns, and tile distribution.
    """
    tiles = {name: rospy.get_param('board_{}_tiles'.format(name)) for name in TILE_TYPES}
    return rospy.get_param('board_height'), rospy.get_param('board_width'), tiles


def solve_puzzle_optimized():
    rows, columns, tiles = get_board_state()
    solution = solve_puzzle(rows, columns, tiles)
    assert solution is not None, 'Failed to solve puzzle.'
    solution = optimize_solution(solution)
    print('\n')
    display_solution(rows, columns, solution, scale=4, offset=4)
    print('\n')
    return solution


def add_table_obstacle(task, margin=0.01):
    task.planner.add_box_obstacle([1, 1, 0.5], 'table', )


def main():
    rospy.init_node('tetris')
    verbose = rospy.get_param('verbose').lower() == 'true'
    solution = solve_puzzle_optimized()

    # task = TetrisPNPTask(verbose=verbose)

    # while not rospy.is_shutdown():
    #     raw_input('Press enter.')

    # solver = TetrisSolver(board_rows=6, board_cols=8,
    # tiles=[SquareTile, LTile, ReverseLTile, TTile, LineTile, ZTile, STile], num_tiles=[2, 2, 2, 1, 2, 2, 1])


if __name__ == '__main__':
    try:
        main()
    except exc:
        rospy.logerr(str(exc))
