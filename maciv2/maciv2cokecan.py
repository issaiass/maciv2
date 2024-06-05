import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.srv import GetPlanningScene


arm_joints = ['ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint',
              'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint']
gripper_joints = ['finger_joint']
move_near_can = [-0.0015519098115023841, -1.4166410229735018, 1.4751473434823434,
                 -3.24299592498238,  9.054329044174694e-05, 0.07470218135193267]
approach = [0.17812759305374887, -1.351754424564953, 1.3975273326515718,
            -3.187367787619521, -0.17947340985238477, 0.03185348315391471]
close_gripper = [-0.05]
move_a_little_up = [0.17856525056704994, -1.3464446120968898, 1.4837025627689286,
                    -3.278695330575958, -0.179908489362678, 0.03184251976800694]
move_up = [0.15050512021677948, -1.2080349055387634, 1.8237752322274088,
           -3.7574097631517294, -0.15198210263835593, 0.03192701474892201]
move_border = [0.5083103262881384, -1.9314110982015147, 2.5131769575227243,
               -3.723400262262758, -0.5097826953572108, 0.031924918851899006]
move_a_little_down = [0.50346967635467, -2.022675891322579, 2.393631126061756,
                      -3.5213799077333663, -0.5046882683161481, 0.03157445613394978]
move_near_table = [0.4918558996736666, -2.0909656810645343, 2.0753470488608095,
                   -3.1259440571334394, -0.49318420818402636, 0.03184831222336501]
open_gripper = [0.00]
retire = [0.5175658600766134, -1.582214101825012, 2.673728271258786,
          -4.233029027046448, -0.5189729894321163, 0.03193792843314274]

move_home = 6*[0]

manipulator_actions = {
    'open_gripper1': dict(message='Open Gripper', positions=open_gripper, joint_names=gripper_joints, duration=1, node='gripper'),    
    'move_near_can': dict(message='Move Down In front of Coke can', positions=move_near_can, joint_names=arm_joints, duration=6, node='arm'),
    'approach': dict(message='Approach to Coke can', positions=approach, joint_names=arm_joints, duration=3, node='arm'),
    'close_gripper': dict(message='Close the gripper', positions=close_gripper, joint_names=gripper_joints, duration=1, node='gripper'),
    'move_a_little_up': dict(message='Move a little Up the Coke can', positions=move_a_little_up, joint_names=arm_joints, duration=3, node='arm'),
    'move_up': dict(message='Move Up the Coke can', positions=move_up, joint_names=arm_joints, duration=3, node='arm'),
    'move_border': dict(message='Move to the border of the table', positions=move_border, joint_names=arm_joints, duration=3, node='arm'),
    'move_a_little_down': dict(message='Move a little bit down', positions=move_a_little_down, joint_names=arm_joints, duration=3, node='arm'),
    'move_near_table': dict(message='Move down near table', positions=move_near_table, joint_names=arm_joints, duration=3, node='arm'),
    'open_gripper2': dict(message='Open Gripper', positions=open_gripper, joint_names=gripper_joints, duration=1, node='gripper'),
    'retire': dict(message='Retire or move up', positions=retire, joint_names=arm_joints, duration=3, node='arm'),
    'move_home': dict(message='Move To Home Position', positions=move_home, joint_names=arm_joints, duration=3, node='arm'),
}


def control_joints(joint_names, positions):
    traj = JointTrajectory()
    traj.joint_names = joint_names
    
    traj.points =  [JointTrajectoryPoint(positions=positions)]
    if len(traj.joint_names) > 1: # if not the gripper (gripper has 1 joint)
        traj.points =  [JointTrajectoryPoint(positions=positions, velocities=6*[0.001])]
   
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    goal.goal_time_tolerance = Duration(sec=1)
    
    tol = [JointTolerance(name=n, position=0.001, velocity=0.001) for n in traj.joint_names]
    
    goal.path_tolerance = tol
    goal.goal_tolerance = tol

    return goal

def connect(name:str):
    node = Node(f"{name}_controller")
    logger = node.get_logger()
    action_client = ActionClient(node, FollowJointTrajectory, f"/{name}_controller/follow_joint_trajectory")
    logger.info(f"Connected to {name} server")
    action_client.wait_for_server()
    return action_client, node, logger

def main():
    rclpy.init()

    arm_ac, arm_node, arm_logger = connect('arm')
    gripper_ac, gripper_node, gripper_logger = connect('gripper')

    for state, state_dict in manipulator_actions.items():
        joint_names = state_dict['joint_names']
        positions = state_dict['positions']
        goal = control_joints(joint_names, positions)
        if state_dict['node'] == 'arm':
            action_client, node, logger = arm_ac, arm_node, arm_logger
        if state_dict['node'] == 'gripper':
            action_client, node, logger = gripper_ac, gripper_node, gripper_logger
        logger.info(f"{state}: {state_dict['message']}")
        future = action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future)
        time.sleep(state_dict['duration'])

    rclpy.shutdown()


if __name__ == '__main__':
    main()
