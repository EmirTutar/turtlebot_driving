#!/usr/bin/env python3

import rospy
from goals import GoalsList

if __name__ == '__main__':
    try:
        rospy.init_node("final_project")

        rospy.loginfo("main - Starting final project...")

        goals_list = GoalsList()

        rospy.loginfo("main - Goals loaded!")

        goals_list.process_goals()

        rospy.loginfo("main - Goals processed!")


        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("main - ROS Interrupt Exception!")
        pass
