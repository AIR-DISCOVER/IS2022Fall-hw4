import sys
import rospy
from math import pi
from geometry_msgs.msg import PoseStamped
from tf_conversions import transformations

if __name__ == "__main__":
    rospy.init_node('setgoal', anonymous=True)
    points = [[ 0.10,  2.75,  pi/2],
              [-0.03,  2.35,  0.00],
              [ 2.05,  2.68,  0.00],
              [ 2.13,  0.21, -pi  ],
              [ 2.76, -0.80,  0.00]]

    if len(sys.argv) > 1:
        if len(sys.argv[1]) == 1 and "0" < sys.argv[1] < "6":
            idx = int(sys.argv[1])
            goal_puber = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
            simple_goal = PoseStamped()
            simple_goal.header.stamp = rospy.Time.now()
            simple_goal.header.frame_id = "map"
            simple_goal.pose.position.x = points[idx-1][0]
            simple_goal.pose.position.y = points[idx-1][1]
            simple_goal.pose.position.z = 0.0
            quat = transformations.quaternion_from_euler(0.0, 0.0, points[idx-1][2])
            simple_goal.pose.orientation.x = quat[0]
            simple_goal.pose.orientation.y = quat[1]
            simple_goal.pose.orientation.z = quat[2]
            simple_goal.pose.orientation.w = quat[3]
            for i in range(3):
                goal_puber.publish(simple_goal)
                rospy.sleep(0.2)
            print("pub goal ID = {}".format(idx))

        else:
            print("Invalid arg")
            quit()
    else:
        print("Invalid arg")
        quit()
