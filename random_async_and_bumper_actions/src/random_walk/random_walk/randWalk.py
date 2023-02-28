import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from geometry_msgs.msg import PoseStamped
from action_msgs.msg._goal_status import GoalStatus

import irobot_create_msgs
from irobot_create_msgs.action import DriveDistance, Undock, RotateAngle, Dock, NavigateToPosition
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.srv import ResetPose

from pynput.keyboard import KeyCode
from key_commander import KeyCommander
from threading import Lock
from rclpy.executors import MultiThreadedExecutor

import random

# To help with Multithreading
lock = Lock()

class Slash(Node):
    """
    Class to coordinate actions and subscriptions
    """

    def __init__(self, namespace):
        super().__init__('slasher')

        # 2 Seperate Callback Groups for handling the bumper Subscription and Action Clients
        cb_Subscripion = MutuallyExclusiveCallbackGroup()
        cb_Action = MutuallyExclusiveCallbackGroup()

        # Subscription to Hazards, the callback function attached only looks for bumper hits
        self.subscription = self.create_subscription(
            HazardDetectionVector, f'/{namespace}/hazard_detection', self.listener_callback, qos_profile_sensor_data,callback_group=cb_Subscripion)

        # Action clients for movements
        self._undock_ac = ActionClient(self, Undock, f'/{namespace}/undock',callback_group=cb_Action)
        self._dock_ac = ActionClient(self, Dock, f'/{namespace}/dock',callback_group=cb_Action)
        self._drive_ac = ActionClient(self, DriveDistance, f'/{namespace}/drive_distance',callback_group=cb_Action)
        self._rotate_ac = ActionClient(self, RotateAngle, f'/{namespace}/rotate_angle',callback_group=cb_Action)
        self._nav_ac = ActionClient(self, NavigateToPosition, f'/{namespace}/navigate_to_position',callback_group=cb_Action)

        self._reset_sc = self.create_client( ResetPose, f'/{namespace}/reset_pose')
        self.req = ResetPose.Request()

        # Variables
        self._goal_uuid = None
        self._pose = None
        self._bump = 0


    def listener_callback(self, msg):
        '''
        This function is called every time self.subscription gets a message
        from the Robot. Here it parses the message from the Robot and if its
        a 'bump' message, cancel the current action and move away from object. 

        For this to work, make sure you have:
        ros__parameters:
            reflexes_enabled: false
        in your Application Configuration Parameters File
        '''

        # If it wasn't doing anything, there's probably nothing to get out of the way of
        if self._goal_uuid is None:
            return

        # msg.detections is an array of HazardDetection from HazardDetectionVectors.
        # Other types can be gotten from HazardDetection.msg
        for detection in msg.detections:
            if detection.type == 1:   #If it is a bump
                # This variable stops further execution of other actions on goal cancel
                self._bump = 1 
                self.get_logger().warning('HAZARD DETECTED')
                with lock: # Make this the only thing happening
                    self.get_logger().warning('GOAL ID NOT NONE. Cancelling goal')           
                    self._goal_uuid.cancel_goal_async()
                    # Loop until goal status returns canceled
                    while self._goal_uuid.status is not GoalStatus.STATUS_CANCELED:
                        pass    
                    print('Goal canceled.')
    
                # create goal object and specify angle (in radians!)
                print('Rotating...')
                rotate_goal = RotateAngle.Goal()
                rotate_goal.angle = 3.142 # ~180 Degreees

                print("WAITING FOR ACTION SERVER TO ROTATE")
                self._rotate_ac.wait_for_server()
                rotate_handle = self._rotate_ac.send_goal_async(rotate_goal) # non Blocking
                
                while not rotate_handle.done():
                    pass # Wait until Action Server accepts goal
                
                # Keep track of the Status of the goal until it has succeeded
                while rotate_handle.result().status is not GoalStatus.STATUS_SUCCEEDED:
                    pass # Loop until it's succeeded

                # create goal object and specify distance to drive (in meters!)
                print('Driving away from object...')
                drive_goal = DriveDistance.Goal()
                drive_goal.distance = 0.5

                print("WAITING FOR ACTION SERVER TO Drive")
                self._drive_ac.wait_for_server()
                drive_handle = self._drive_ac.send_goal_async(drive_goal) # Not Blocking

                while not drive_handle.done():
                    pass # Wait until Action Server accepts goal
                
                #print(drive_handle.result().status)
                # Keep track of the Status of the goal until it has succeeded
                while drive_handle.result().status is not GoalStatus.STATUS_SUCCEEDED:
                    pass # Loop until it's succeeded

                self._bump = 0 # If something was waiting, let it through now
                
                self.get_logger().warning('Reflex complete, going back to function')


#--------------------Async send goal calls-----------------------------
    def sendDriveGoal(self,goal):
        """
        Sends a drive goal asynchronously and 'blocks' until the goal is complete
        """
        #print("Acquiring lock")
        with lock:
            drive_handle = self._drive_ac.send_goal_async(goal)
            # print('Goal Sent.')
            while not drive_handle.done():
                pass # Wait for Action Server to accept goal
            # print('ID gotten.')
            self._goal_uuid = drive_handle.result() # Hold ID in case we need to cancel it
            # print('ID Assigned.')
        # print("lock released")

        # print("idling because status is unknown.")
        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            pass # Wait until a Status has been assigned
        #print(self._goal_uuid.status)

        status = self._goal_uuid.status
        # After getting goalID, block until goal is done
        while (self._bump == 1 or(self._goal_uuid.status == GoalStatus.STATUS_EXECUTING or self._goal_uuid.status == GoalStatus.STATUS_CANCELING)):
            # Loop while bumper actions are running or while the goal is currently running
            if(self._goal_uuid.status is not status):
                #print(self._goal_uuid.status) # Print new Statuses
                status = self._goal_uuid.status

            if self._goal_uuid is None or self._goal_uuid is GoalStatus.STATUS_CANCELED:
                break # If the goal is done or was canceled, stop looping
            pass
            
        # print(self._goal_uuid.status)

        while self._bump == 1:
                pass # If the bumper actions are running, dont cancel those goals! Wait to be done
        self._goal_uuid = None # Reset the goal ID, nothing should be running
        #print('cleared UUID')


    def sendTurnGoal(self,goal):
        """
        Sends a rotate goal asynchronously and 'blocks' until the goal is complete
        """
        #print("Acquiring lock")
        with lock:
            drive_handle = self._rotate_ac.send_goal_async(goal)
            # print('Goal Sent.')
            while not drive_handle.done():
                pass # Wait for Action Server to accept goal
            # print('ID gotten.')
            self._goal_uuid = drive_handle.result() # Hold ID in case we need to cancel it
            # print('ID Assigned.')
        # print("lock released")

        # print("idling because status is unknown.")
        while self._goal_uuid.status == GoalStatus.STATUS_UNKNOWN:
            pass # Wait until a Status has been assigned
        #print(self._goal_uuid.status)

        status = self._goal_uuid.status
        # After getting goalID, block until goal is done
        while (self._bump == 1 or(self._goal_uuid.status == GoalStatus.STATUS_EXECUTING or self._goal_uuid.status == GoalStatus.STATUS_CANCELING)):
            # Loop while bumper actions are running or while the goal is currently running
            if(self._goal_uuid.status is not status):
                #print(self._goal_uuid.status) # Print new Statuses
                status = self._goal_uuid.status

            if self._goal_uuid is None or self._goal_uuid is GoalStatus.STATUS_CANCELED:
                break # If the goal is done or was canceled, stop looping
            pass
            
        # print(self._goal_uuid.status)

        while self._bump == 1:
                pass # If the bumper actions are running, dont cancel those goals! Wait to be done
        self._goal_uuid = None # Reset the goal ID, nothing should be running
        #print('cleared UUID')
#----------------------------------------------------------------------


    def randWalk(self):
        """
        Undocks robot and takes its position. Repeatedly calls DoWalk() to complete 
        random manuvers; afterwhich it will return to the captured position and redock
        """
        # Freshly started, undock
        while self._bump == 1:
            pass # If bump callback is running, wait for that to be done first
        self.get_logger().warning('WAITING FOR SERVER')
    # wait until the robot server is found and
    #   ready to receive a new goal
        self._undock_ac.wait_for_server()
        self.get_logger().warning('SERVER AVAILABLE')
        self.get_logger().warning('UNDOCKING')

    # create new Undock goal object to send to server
        undock_goal = Undock.Goal()

        self._undock_ac.send_goal(undock_goal)
        self.get_logger().warning('UNDOCKED')

    # Drive
    # wait for DriveDistance action server (blocking)
        self._drive_ac.wait_for_server()
        self.get_logger().warning('DRIVING!')

    # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 1.0

        while self._bump == 1:
            pass # If bump callback is running, wait for that to be done first
        self.sendDriveGoal(drive_goal)

    # Get positional data 
        while self._bump == 1:
            pass # If bump callback is running, wait for that to be done first
        self._reset_sc.call(self.req) # Reset the pose 
        self._pose = PoseStamped() # Capture new pose
        self.get_logger().warning('POSE CAPTURED')

    # Do loop
        # Start walking loop (10 walks)
        for i in range(0,10): # Needs 10
            while self._bump == 1:
                pass # If bump callback is running, wait for that to be done first
            self.DoWalk(i)

    # Navigate back to Dock
        while True:
            # Return (roughly) to pose capture
            while self._bump == 1:
                pass # If bump callback is running, wait for that to be done first
            nav_goal = NavigateToPosition.Goal()
            nav_goal.goal_pose = self._pose

            self.get_logger().warning('RETURNING TO DOCK')
            while self._bump == 1:
                pass # If bump callback is running, wait for that to be done first
            # If bumper hit here, bumper callback will 'bully' the AC and overwrite the goal
            self._nav_ac.send_goal(nav_goal) 
            # Check that the goal actually completed, if not loop and try again
            if nav_goal.remaining_travel_distance == 0 and nav_goal.remaining_angle_travel == 0:
                break

    # Redock
        # create new Dock goal object to send to server
        dock_goal = Dock.Goal()
        while self._bump == 1:
            pass # If bump callback is running, wait for that to be done first
        # send the goal (blocking)
        self._dock_ac.send_goal(dock_goal) 
        self.get_logger().warning('DOCKED')



    def DoWalk(self, count):
        msg = 'Walk number: ' + str(count+1)
        self.get_logger().warning(msg)
    # Grab a random number between 0.00 - 6.28 (for radians to turn)
        radians = round(random.uniform(0,6.28),2) 
        msg = 'Turning radians: ' + str(radians)
        self.get_logger().warning(msg)

    # create goal object and specify angle (using random angle from before)
        rotate_goal = RotateAngle.Goal()
        rotate_goal.angle = radians

        self.sendTurnGoal(rotate_goal)

    # create goal object and specify distance to drive
        drive_goal = DriveDistance.Goal()
        drive_goal.distance = 0.75

        self.sendDriveGoal(drive_goal)



if __name__ == '__main__':
    rclpy.init()

    namespace = 'create3_05AE'
    s = Slash(namespace)

    # 1 thread for the Subscription, another for the Action Clients
    exec = MultiThreadedExecutor(2)
    exec.add_node(s)

    keycom = KeyCommander([
        (KeyCode(char='r'), s.randWalk),
        ])

    print("r: Start Random Walk")
    try:
        exec.spin() # execute slash callbacks until shutdown or destroy is called
    except KeyboardInterrupt:
        s.get_logger().info('KeyboardInterrupt, shutting down.\n')
    rclpy.shutdown()
