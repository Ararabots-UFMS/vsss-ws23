#!/usr/bin/python3
from ast import arg
from logging import debug
import rclpy
import sys
from argparse import ArgumentParser
from robot.robot import Robot
import os
from utils.debug_profile import debug_profiler


def makeArgParser() -> ArgumentParser:
    parser = ArgumentParser(prefix_chars="--")
    parser.add_argument("id", type=int, help="robot id")
    parser.add_argument("tag", type=int, help="tag id")
    parser.add_argument("body", type=str, help="robot body name")
    parser.add_argument("team_side", type=int, help="robot team side: 0->---, 1->---")
    parser.add_argument("team_color", type=int, help="robot team color: 0->blue, 1->yellow")
    parser.add_argument("robot_role", type=int, help="robot state machine id")
    parser.add_argument("owner_name", type=str, help="owner of this game topic and robots")
    parser.add_argument("socket_id", type=int, help="robot socket id")
    parser.add_argument("should_debug", type=int, default=0, help="robot debug flag")
    parser.add_argument("ros-args", nargs='*', help="additional ros parameters")
    return parser


def main(_args=None):
    rclpy.init(args=_args)
    
    #parser = makeArgParser()
    #args = parser.parse_args()

    # rospy.logfatal(sys.argv)
    

    #robot = Robot(node_obj, args.id, args.tag, args.body, args.team_side, args.team_color,
    #              args.robot_role, args.owner_name, args.socket_id, args.should_debug)

    node_obj = Robot(int(sys.argv[1]), int(sys.argv[2]), sys.argv[3], int(sys.argv[4]), int(sys.argv[5]),
                int(sys.argv[6]), sys.argv[7], int(sys.argv[8]), int(sys.argv[9]))

    #rospy.init_node(robot_name, anonymous=True)    
    
    node_obj.get_logger().fatal("ROBOT_" + str(node_obj.id) + " TAG: " + str(sys.argv[2]) + " Online")


    rclpy.spin(node_obj)

    node_obj.destroy_node()
    rclpy.shutdown()
    
    # debug_profiler.dump_stats(os.environ["ROS_ARARA_ROOT"] + "debug_logs.bin") # TODO: pensar em uma forma boa de habilitar isso


if __name__ == '__main__':
    main()
