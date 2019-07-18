import rospy
from gazebo_msgs.msg import LinkStates 
import numpy as np
import thread

lock = thread.allocate_lock()

def callback(msg):
    global prev_vel
    global accelerations
    global times
    global i
    global lock
    lock.acquire()
    index = i
    if index >= 10000:
		return
    i += 1
    lock.release()
    times[index] = rospy.get_time()
    vel = msg.twist[11].linear.z
    accelerations[index] = (vel - prev_vel)
    prev_vel = vel

prev_vel = 0
i = 0
accelerations = np.zeros((10000,))
times = np.zeros((10000,))
rospy.init_node("probe_acceleration", anonymous=True)
rospy.Subscriber("/gazebo/link_states", LinkStates, callback)

rospy.sleep(2)


"""
Observations: only for linear.z significant accelerations are found which is inline with the differences between the bumper and ft sensor which are also mainly in z
"""
