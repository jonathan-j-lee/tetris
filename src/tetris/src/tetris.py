#!/usr/bin/env python

"""
tetris -- Solves a Tetris-like puzzle.
"""

from __future__ import division, generators, print_function
import os
from baxter_interface import Limb
import cv2
import cv_bridge
import rospy
import rospkg
from sensor_msgs.msg import Image
from pnp import TetrisPNPTask
from solver import TILE_TYPES, solve_puzzle, optimize_solution, display_solution

# JOINTS = [-1.5834516682947184, 1.6256361399615673, -0.11543205428837738, 0.5763932810479442, -0.9660244011708393, -1.571179821991635, 2.073558530024108]
# JOINTS = [-1.6551652701283615, 1.5240099127641586, 0.0, 0.14611167004608566, -1.3959225169757266, -1.5707963267946636, 2.3109420569493757]
JOINTS = [-1.4304370847031482, 1.4814419459003383, -0.32098547986502285, -0.7765777738669907, -2.3749857548435918, -1.5715633171886063, 2.1517915502062643]
INIT_LEFT_ARM_JOINTS = {
    'left_e0': JOINTS[0],
    'left_e1': JOINTS[1],
    'left_s0': JOINTS[2],
    'left_s1': JOINTS[3],
    'left_w0': JOINTS[4],
    'left_w1': JOINTS[5],
    'left_w2': JOINTS[6],
}


def send_image(img_pub, path, delay=1):
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding='bgr8')
    img_pub.publish(msg)
    rospy.sleep(delay)


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


def set_joint_angles(limb, angles, tolerance=0.05, max_steps=1000, period=0.01):
    joint_names = limb.joint_names()
    all_angles = {name: (angles[name] if name in angles else limb.joint_angle(name))
                  for name in joint_names}
    step = 0
    while step < max_steps and not all(abs(angle - limb.joint_angle(name)) < tolerance for name, angle in all_angles.items()):
        limb.set_joint_positions(all_angles)
        rospy.sleep(period)
        step += 1


# def add_table_obstacle(task, margin=0.01):
#     task.planner.add_box_obstacle([1, 1, 0.5], 'table', )


def main():
    rospy.init_node('tetris')
    # Wait for other nodes to come online.
    rospy.sleep(rospy.get_param('init_delay'))
    set_joint_angles(Limb('left'), INIT_LEFT_ARM_JOINTS)
    rospack = rospkg.RosPack()
    solution, i = solve_puzzle_optimized(), 0
    task = TetrisPNPTask()
    img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=5)

    while not rospy.is_shutdown() and i < len(solution):
        tile = solution[i]
        prompt = '(Please provide a {} tile. Press enter when done.) '

        # try:
        path = os.path.join(rospack.get_path('tetris'),
                                'data/{}.png'.format(tile.tile_name))
        send_image(img_pub, path)
        raw_input(prompt.format(tile.tile_name.upper()))
        task.pick(tile.tile_name)
        task.place(tile)
        # except Exception as exc:
        #     # Retry the current piece in the event of an error
        #     rospy.logerr(type(exc).__name__ + ': ' + str(exc))
        #     continue
        #     raise exc
        # else:
        i += 1


if __name__ == '__main__':
    main()
