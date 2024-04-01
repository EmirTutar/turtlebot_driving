import rospy
import yaml
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_srvs.srv import Empty

class Goal:
    def __init__(self, id, x, y, reward, zone):
        self.id = id
        self.x = x
        self.y = y
        self.reward = reward
        self.zone = zone

    def create_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0
        return pose

    def getInfoString(self):
        return f"id: {self.id} x: {self.x} y: {self.y} reward: {self.reward} zone: {self.zone}"

class GoalsList:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.goals = self.load_goals()
        self.temp_unreachable_goals = []  
        self.current_position = None
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.last_goal_retry = False
        self.clear_costmap_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

    def load_goals(self):
        config_file_name = rospy.get_param("~goals_config_file", "goals.yaml2")
        rospy.loginfo(f"Loading configuration file: {config_file_name}")
        try:
            with open(config_file_name, "r") as ymlfile:
                configFile = yaml.load(ymlfile, yaml.SafeLoader)
                return self.extract_goals(configFile["goals"])
        except Exception as e:
            rospy.logerr(f"Could not open or extract file {config_file_name}: {e}")
            return []

    def amcl_pose_callback(self, msg):
        position = msg.pose.pose.position
        self.current_position = Goal(id=-1, x=position.x, y=position.y, reward=0, zone="current")

    def extract_goals(self, goals_yaml):
        goals = []
        for key, value in goals_yaml.items():
            goal = Goal(id=key, x=value['x'], y=value['y'], reward=value['reward'], zone=value['zone'])
            goals.append(goal)
            rospy.loginfo(f"Added goal: {goal.getInfoString()}")
        return goals

    def process_goals(self):
        while len(self.goals) > 0 or len(self.temp_unreachable_goals) > 0:
            if len(self.goals) == 0 and len(self.temp_unreachable_goals) == 1 and not self.last_goal_retry:
                self.perform_side_movement()
                self.last_goal_retry = True 
                self.goals += self.temp_unreachable_goals
                self.temp_unreachable_goals = []
            
            goal = self.select_next_goal()
            if goal is None:
                rospy.logwarn("No more goals to process.")
                break
            rospy.loginfo(f"Processing goal: {goal.getInfoString()}")
            self.send_goal(goal.create_pose(), goal)

    def calculate_distance(self, goal):
        if self.current_position is None:
            return float('inf')
        return ((goal.x - self.current_position.x) ** 2 + (goal.y - self.current_position.y) ** 2) ** 0.5

    def select_next_goal(self):
        if len(self.goals) == 0 and len(self.temp_unreachable_goals) > 0:
            self.goals += self.temp_unreachable_goals
            self.temp_unreachable_goals = []

        if len(self.goals) > 0:
            self.goals.sort(key=self.calculate_distance)
            return self.goals.pop(0)
        return None


    def send_goal(self, pose, goal, timeout_duration=20.0):
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = pose.header.frame_id
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose = pose.pose

        self.client.send_goal(move_base_goal)
        finished_within_time = self.client.wait_for_result(rospy.Duration(timeout_duration))

        if not finished_within_time or self.client.get_state() != actionlib.GoalStatus.SUCCEEDED:
            rospy.logwarn(f"Goal {goal.id} not reached within time limit or failed. Attempting to clear costmaps and retry.")
            try:
                self.clear_costmap_service.call()
                rospy.loginfo("Costmaps cleared in an attempt to resolve the issue.")
            except rospy.ServiceException as e:
                rospy.logerr("Failed to clear costmaps: %s", e)
            self.temp_unreachable_goals.append(goal)
        else:
            rospy.loginfo(f"Goal {goal.id} successfully reached!")
            rospy.sleep(2)
            self.goals += self.temp_unreachable_goals
            self.temp_unreachable_goals = []
