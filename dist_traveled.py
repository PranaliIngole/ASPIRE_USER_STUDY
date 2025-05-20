# import rosbag
# import pandas as pd

# # Path to your recorded ROS bag file
# #bag_file = '/home/inspire_lab/Documents/user_data_com/user_data/LCM_2_26_05_24/User_2_Task_10_Attempt_5_counter.bag'  # Replace with your bag file path
# bag_file = '/home/inspire_lab/ASPIRE_F/user_study/THD/User_2_Task_10_Attempt_5_counter.bag'
# output_csv_file = 'User_2_Task_10_Attempt_5_counter.bag.csv'  # Output CSV file name

# # Initialize an empty list to store data
# data = []

# # Open the ROS bag file
# with rosbag.Bag(bag_file, 'r') as bag:
#     for topic, msg, t in bag.read_messages():
#         # Append a dictionary of the message data to the list
#         data.append({
#             'topic': topic,
#             'timestamp': t.to_sec(),
#             'message': str(msg)  # Convert the message to a string
#         })

# # Create a DataFrame from the list of data
# df = pd.DataFrame(data)

# # Save the DataFrame to a CSV file
# df.to_csv(output_csv_file, index=False)

# print(f"Data saved to {output_csv_file}")




#!/usr/bin/env python3
import rospy
import rosbag
import moveit_commander
from tf_conversions import posemath
# from PyKDL import Chain, JntArray, Joint
# from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
from std_msgs.msg import Float64MultiArray
# from iiwa_msgs.msg import task
#from custom_messages.msg import ToolMasterControlData
import numpy as np
import math
from math import sqrt,sin,cos,atan,atan2
import tf2_ros
import tf2_geometry_msgs
from tf import transformations,TransformListener
# from custom_messages.msg import CurrentPos
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

latency = 0 
teleop_start_time = None

def calculate_distance(p1, p2):
   
    return (np.linalg.norm(np.array(p2))- np.linalg.norm(np.array(p1)))
def calculate_latency(master_start_time, slave_start_time):
    return master_start_time - slave_start_time


rospy.init_node('fk_calculator')


compute_fk_service = rospy.ServiceProxy('/iiwa/compute_fk', GetPositionFK)

tele_on=False
slave_positions = []
master_positions = []
slave_distances = []
master_distances = []
slave_timestamps = []
master_timestamps = []
error = 0
count=0
count1=0
count2=0
norm1=0
prev_x = 0
prev_y = 0
prev_z = 0
norm_x = 0
norm_y = 0
norm_z = 0
total_count=0
initial_master_x = None
initial_slave_x = None
initial_master_y = None
initial_slave_y = None
initial_master_z = None
initial_slave_z = None
initial_slave_time=None
initial_master_time=None
total_data_loss=0
data_0=0
data_sent=0
bag_path = 'LCM_2_26_05_24/User_2_Task_1_Attempt_1_counter.bag'
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, timestamp in bag.read_messages():
        
       
        
        if topic == '/hardware_kuka_joint_values':
            
            
            request = GetPositionFKRequest()
            request.header.stamp = rospy.Time.now()
            request.header.frame_id = "iiwa_link_0"
            request.fk_link_names = ["tool_link_eee"]
            request.robot_state.joint_state.name = [
                "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3",
                "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"
            ]
            request.robot_state.joint_state.position = msg.data

            response = compute_fk_service(request)

            if response.pose_stamped:
            
                position = response.pose_stamped[0].pose.position
                x, y, z = position.x, position.y, position.z
                # count1+=1
                #if tele_on:
                    #print(tele_on)
                    
                if initial_slave_time is None:
                    initial_slave_time=timestamp.to_sec()
                slave_timestamps.append(timestamp.to_sec()-initial_slave_time)
                
                slave_start_time = timestamp.to_sec()
                if initial_slave_x is None:
                    initial_slave_x = x  # Store the initial "slave" x-coordinate
                    initial_slave_y = y
                    initial_slave_z = z
        
                slave_positions.append([ x*1000- initial_slave_x*1000, y*1000-initial_slave_y*1000,z*1000-initial_slave_z*1000])#
                #slave_positions.append([ x- initial_slave_x, y-initial_slave_y,z-initial_slave_z])# 
        if topic == '/iiwa/target_pos':
            x, y, z = msg.x,msg.y,msg.z
            # print(msg.data[3])
            # print("dddddddd")
            #if msg.data[3] == 1.0:
            if initial_master_time is None:
                initial_master_time=timestamp.to_sec()
            master_timestamps.append(timestamp.to_sec()-initial_master_time)
            #count+=1
            if initial_master_x is None:
                initial_master_x = x  # Store the initial "master" x-coordinate
                initial_master_y = y
                initial_master_z = z
                # print(x,y,z)
            
            master_positions.append([x-initial_master_x, y-initial_master_y, z-initial_master_z])#
            
            master_start_time = timestamp.to_sec()
            
        # if topic == '/iiwa/target_pos':
            # print(msg.x)
            
                
                    
            

for i in range(1, len(slave_positions)):
    slave_distance = calculate_distance(slave_positions[i-1], slave_positions[i])
   
    slave_distances.append(slave_distance)

for i in range(1, len(master_positions)):
    
    master_distance = calculate_distance(master_positions[i-1], master_positions[i])
    
    master_distances.append(master_distance)

total_slave_distance = sum(slave_distances)
total_master_distance = sum(master_distances)


print(f"Total distance traveled by slave: {total_slave_distance} millimeters")
print(f"Total distance traveled by master: {total_master_distance} millimeters")


# Calculate cumulative distances for plotting
# cumulative_slave_distance = np.cumsum(slave_distances)
# cumulative_master_distance = np.cumsum(master_distances)

# Plot the cumulative distances
# plt.figure(figsize=(10, 6))
# plt.plot(slave_timestamps[1:], cumulative_slave_distance, label="Slave (KUKA) Cumulative Distance")
# plt.plot(master_timestamps[1:], cumulative_master_distance, label="Master Cumulative Distance")
# plt.title("Cumulative Distance Traveled by Master and Slave")
# plt.xlabel("Time (s)")
# plt.ylabel("Cumulative Distance (mm)")
# plt.legend()
# plt.grid()
# plt.show()


# slave_positionss = []
# master_positionss = []

# for inner_list in slave_positions:
#     slave_positionss.append(inner_list[0])
 

# for inner_list in master_positions:
#     master_positionss.append(inner_list[0])
# time_slave = np.arange(len(slave_positionss))
# time_master = np.arange(len(master_positionss))




# #plt.plot(slave_timestamps, slave_positions, label="Slave (KUKA) Position")
# plt.plot(master_timestamps, master_positions, label="Master Position")
# plt.title("Position of Slave (KUKA) and Master")
# plt.xlabel("Time Step")
# plt.ylabel("Position (x, y, z)")
# plt.legend()
# plt.show()

slave_positionss_x = [pos[0] for pos in slave_positions]
slave_positionss_y = [pos[1] for pos in slave_positions]
slave_positionss_z = [pos[2] for pos in slave_positions]

master_positionss_x = [pos[0] for pos in master_positions]
master_positionss_y = [pos[1] for pos in master_positions]
master_positionss_z = [pos[2] for pos in master_positions]

time_slave = np.arange(len(slave_positionss_x))
time_master = np.arange(len(master_positionss_x))

# Create 3D plot
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Plot Slave Trajectory with x, y, z in different colors
ax.plot(slave_positionss_x, slave_positionss_y, slave_positionss_z, color="blue", label="Slave Trajectory (x, y, z)")

# Plot Master Trajectory with x, y, z in different colors
# ax.plot(master_positionss_x, master_positionss_y, master_positionss_z, color="orange", label="Master Trajectory (x, y, z)")

# Add labels and title
ax.set_title("3D Line Plot of Slave and Master Positions")
ax.set_xlabel("X Position")
ax.set_ylabel("Y Position")
ax.set_zlabel("Z Position")
ax.legend()

# Show plot
plt.show()