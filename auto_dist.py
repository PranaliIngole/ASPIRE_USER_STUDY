
# #!/usr/bin/env python3
# import rospy
# import rosbag
# import moveit_commander
# from tf_conversions import posemath
# # from PyKDL import Chain, JntArray, Joint
# # from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
# from std_msgs.msg import Float64MultiArray
# # from iiwa_msgs.msg import task
# #from custom_messages.msg import ToolMasterControlData
# import numpy as np
# import math
# from math import sqrt,sin,cos,atan,atan2
# import tf2_ros
# import tf2_geometry_msgs
# from tf import transformations,TransformListener
# # from custom_messages.msg import CurrentPos
# from sensor_msgs.msg import JointState
# from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
# from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import os
# import pandas as pd



# rospy.init_node('fk_calculator')


# compute_fk_service = rospy.ServiceProxy('/iiwa/compute_fk', GetPositionFK)

# def calculate_distance(p1, p2):
   
#     return (np.linalg.norm(np.array(p2))- np.linalg.norm(np.array(p1)))
# def calculate_latency(master_start_time, slave_start_time):
#     return master_start_time - slave_start_time


# # Define folder structure and output file paths
# base_folder = "user_1_LCM_25_05_24"
# output_csv_file = "user_1_LCM_25_05_24.csv"
# plot_folder = "plots"

# # Create a folder to save plots if it doesn't exist
# os.makedirs(plot_folder, exist_ok=True)

# # Prepare an empty list to store results for the CSV
# results = []

# # Iterate through all combinations of `a` and `b`
# for a in range(1, 11):  # a ranges from 1 to 5
#     for b in range(1, 6):  # b ranges from 1 to 5
#         bag_file = f"{base_folder}/User_2_Task_{a}_Attempt_{b}_counter.bag"
#         if not os.path.exists(bag_file):
#             print(f"Bag file {bag_file} does not exist. Skipping.")
#             continue

#         print(f"Processing {bag_file}...")

#         # Initialize variables
#         slave_positions = []
#         slave_distances = []
#         slave_timestamps = []
#         initial_master_x = None
#         initial_slave_x = None
#         initial_master_y = None
#         initial_slave_y = None
#         initial_master_z = None
#         initial_slave_z = None
#         initial_slave_time=None
#         initial_master_time=None
#         # Process the .bag file
#         with rosbag.Bag(bag_file, 'r') as bag:
#             for topic, msg, timestamp in bag.read_messages():
#                 if topic == '/hardware_kuka_joint_values':
            
            
#                     request = GetPositionFKRequest()
#                     request.header.stamp = rospy.Time.now()
#                     request.header.frame_id = "iiwa_link_0"
#                     request.fk_link_names = ["tool_link_eee"]
#                     request.robot_state.joint_state.name = [
#                         "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3",
#                         "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"
#                     ]
#                     request.robot_state.joint_state.position = msg.data

#                     response = compute_fk_service(request)

#                     if response.pose_stamped:
                    
#                         position = response.pose_stamped[0].pose.position
#                         x, y, z = position.x, position.y, position.z
#                         # count1+=1
#                         #if tele_on:
#                             #print(tele_on)
                            
#                         if initial_slave_time is None:
#                             initial_slave_time=timestamp.to_sec()
#                         slave_timestamps.append(timestamp.to_sec()-initial_slave_time)
                        
#                         slave_start_time = timestamp.to_sec()
#                         if initial_slave_x is None:
#                             initial_slave_x = x  # Store the initial "slave" x-coordinate
#                             initial_slave_y = y
#                             initial_slave_z = z
                
#                         slave_positions.append([ x*1000- initial_slave_x*1000, y*1000-initial_slave_y*1000,z*1000-initial_slave_z*1000])#
#         # Calculate distances
#         for i in range(1, len(slave_positions)):
#             slave_distance = calculate_distance(slave_positions[i - 1], slave_positions[i])
#             slave_distances.append(slave_distance)

#         # Calculate total slave distance
#         total_slave_distance = sum(slave_distances)
#         print(f"Total distance traveled by slave for {bag_file}: {total_slave_distance} mm")

#         # Append to results
#         results.append({
#             "File": os.path.basename(bag_file),
#             "Total_Distance_mm": total_slave_distance
#         })

#         # Create a 3D plot for the slave trajectory
#         slave_positions_x = [pos[0] for pos in slave_positions]
#         slave_positions_y = [pos[1] for pos in slave_positions]
#         slave_positions_z = [pos[2] for pos in slave_positions]

#         fig = plt.figure()
#         ax = fig.add_subplot(111, projection='3d')
#         ax.plot(slave_positions_x, slave_positions_y, slave_positions_z, color="blue", label="Slave Trajectory")
#         ax.set_title(f"3D Trajectory - Task {a}, Attempt {b}")
#         ax.set_xlabel("X Position (mm)")
#         ax.set_ylabel("Y Position (mm)")
#         ax.set_zlabel("Z Position (mm)")
#         ax.legend()
#         plt.savefig(f"{plot_folder}/Task_{a}_Attempt_{b}.png")
#         plt.close()

# # Save results to a single CSV file
# df = pd.DataFrame(results)
# df.to_csv(output_csv_file, index=False)
# print(f"Results saved to {output_csv_file}")




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
import os
import pandas as pd



rospy.init_node('fk_calculator')


compute_fk_service = rospy.ServiceProxy('/iiwa/compute_fk', GetPositionFK)

def calculate_distance(p1, p2):
   
    return (np.linalg.norm(np.array(p2))- np.linalg.norm(np.array(p1)))
def calculate_latency(master_start_time, slave_start_time):
    return master_start_time - slave_start_time
def calculate_velocity(distance, time):
    return distance / time if time != 0 else 0

def calculate_acceleration(velocity, prev_velocity, time):
    return (velocity - prev_velocity) / time if time != 0 else 0

# Define folder structure and output file paths
base_folder = "user_1_LCM_25_05_24"
output_csv_file = "user_1_LCM_25_05_24_time.csv"
plot_folder = "plots"

# Create a folder to save plots if it doesn't exist
os.makedirs(plot_folder, exist_ok=True)

# Prepare an empty list to store results for the CSV
results = []

# Iterate through all combinations of `a` and `b`
for a in range(1, 11):  # a ranges from 1 to 5
    for b in range(1, 6):  # b ranges from 1 to 5
        bag_file = f"{base_folder}/User_1_Task_{a}_Attempt_{b}_counter.bag"
        if not os.path.exists(bag_file):
            print(f"Bag file {bag_file} does not exist. Skipping.")
            continue

        print(f"Processing {bag_file}...")

        # Initialize variables
        slave_positions = []
        slave_distances = []
        slave_timestamps = []
        slave_velocities = []
        slave_accelerations = []
        initial_master_x = None
        initial_slave_x = None
        initial_master_y = None
        initial_slave_y = None
        initial_master_z = None
        initial_slave_z = None
        initial_slave_time=None
        initial_master_time=None
        # Process the .bag file
        with rosbag.Bag(bag_file, 'r') as bag:
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
        # Calculate distances
        prev_velocity = 0
        for i in range(1, len(slave_positions)):
            slave_distance = calculate_distance(slave_positions[i - 1], slave_positions[i])
            # slave_distances.append(slave_distance)
            time = slave_timestamps[i] - slave_timestamps[i - 1]  # time in seconds
            velocity = calculate_velocity(slave_distance, time)
            acceleration = calculate_acceleration(velocity, prev_velocity, time)
            
            slave_distances.append(slave_distance)
            slave_velocities.append(velocity)
            slave_accelerations.append(acceleration)
            
            prev_velocity = velocity

        # Calculate total slave distance
        
        total_slave_distance = sum(slave_distances)
        total_velocity = sum(slave_velocities) / len(slave_velocities) if slave_velocities else 0
        total_acceleration = sum(slave_accelerations) / len(slave_accelerations) if slave_accelerations else 0
        # Calculate total time
        total_time = slave_timestamps[-1] - slave_timestamps[0] if slave_timestamps else 0
        print(f"Total distance traveled by slave for {bag_file}: {total_slave_distance} mm")
        print(f"Average velocity for {bag_file}: {total_velocity} mm/s")
        print(f"Average acceleration for {bag_file}: {total_acceleration} mm/s²")
        print(f"Total time for {bag_file}: {total_time} s")

        # # Calculate total slave distance
        # total_slave_distance = sum(slave_distances)
        # print(f"Total distance traveled by slave for {bag_file}: {total_slave_distance} mm")

        # Append to results
        # Append to results
        results.append({
            "File": os.path.basename(bag_file),
            "Total_Distance_mm": total_slave_distance,
            "Average_Velocity_mm/s": total_velocity,
            "Average_Acceleration_mm/s2": total_acceleration,
            "Total_Time_s": total_time
        })


        # Create a 3D plot for the slave trajectory
        slave_positions_x = [pos[0] for pos in slave_positions]
        slave_positions_y = [pos[1] for pos in slave_positions]
        slave_positions_z = [pos[2] for pos in slave_positions]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(slave_positions_x, slave_positions_y, slave_positions_z, color="blue", label="Slave Trajectory")
        ax.set_title(f"3D Trajectory - Task {a}, Attempt {b}")
        ax.set_xlabel("X Position (mm)")
        ax.set_ylabel("Y Position (mm)")
        ax.set_zlabel("Z Position (mm)")
        ax.legend()
        plt.savefig(f"{plot_folder}/Task_{a}_Attempt_{b}.png")
        plt.close()

# Save results to a single CSV file
df = pd.DataFrame(results)
df.to_csv(output_csv_file, index=False)
print(f"Results saved to {output_csv_file}")

# #!/usr/bin/env python3
# import rospy
# import rosbag
# import moveit_commander
# from tf_conversions import posemath
# # from PyKDL import Chain, JntArray, Joint
# # from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model
# from std_msgs.msg import Float64MultiArray
# # from iiwa_msgs.msg import task
# #from custom_messages.msg import ToolMasterControlData
# import numpy as np
# import math
# from math import sqrt,sin,cos,atan,atan2
# import tf2_ros
# import tf2_geometry_msgs
# from tf import transformations,TransformListener
# # from custom_messages.msg import CurrentPos
# from sensor_msgs.msg import JointState
# from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
# from moveit_msgs.srv import GetPositionFK, GetPositionFKRequest
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# import os
# import pandas as pd

# rospy.init_node('fk_calculator')

# compute_fk_service = rospy.ServiceProxy('/iiwa/compute_fk', GetPositionFK)

# def calculate_distance(p1, p2):
#     return np.linalg.norm(np.array(p2) - np.array(p1))

# def calculate_velocity(distance, time):
#     return distance / time if time != 0 else 0

# def calculate_acceleration(velocity, prev_velocity, time):
#     return (velocity - prev_velocity) / time if time != 0 else 0

# def calculate_latency(master_start_time, slave_start_time):
#     return master_start_time - slave_start_time

# # Define folder structure and output file paths
# base_folder = "LCM_2_26_05_24"
# output_csv_file = "slave_total_distances_with_velocity_acceleration.csv"
# plot_folder = "plots"

# # Create a folder to save plots if it doesn't exist
# os.makedirs(plot_folder, exist_ok=True)

# # Prepare an empty list to store results for the CSV
# results = []

# # Iterate through all combinations of `a` and `b`
# for a in range(1, 11):  # a ranges from 1 to 5
#     for b in range(1, 6):  # b ranges from 1 to 5
#         bag_file = f"{base_folder}/User_2_Task_{a}_Attempt_{b}_counter.bag"
#         if not os.path.exists(bag_file):
#             print(f"Bag file {bag_file} does not exist. Skipping.")
#             continue

#         print(f"Processing {bag_file}...")

#         # Initialize variables
#         slave_positions = []
#         slave_distances = []
#         slave_velocities = []
#         slave_accelerations = []
#         slave_timestamps = []
#         initial_master_x = None
#         initial_slave_x = None
#         initial_master_y = None
#         initial_slave_y = None
#         initial_master_z = None
#         initial_slave_z = None
#         initial_slave_time = None
#         initial_master_time = None

#         # Process the .bag file
#         with rosbag.Bag(bag_file, 'r') as bag:
#             for topic, msg, timestamp in bag.read_messages():
#                 if topic == '/hardware_kuka_joint_values':
            
            
#                     request = GetPositionFKRequest()
#                     request.header.stamp = rospy.Time.now()
#                     request.header.frame_id = "iiwa_link_0"
#                     request.fk_link_names = ["tool_link_eee"]
#                     request.robot_state.joint_state.name = [
#                         "iiwa_joint_1", "iiwa_joint_2", "iiwa_joint_3",
#                         "iiwa_joint_4", "iiwa_joint_5", "iiwa_joint_6", "iiwa_joint_7"
#                     ]
#                     request.robot_state.joint_state.position = msg.data

#                     response = compute_fk_service(request)

#                     if response.pose_stamped:
                    
#                         position = response.pose_stamped[0].pose.position
#                         x, y, z = position.x, position.y, position.z
#                         # count1+=1
#                         #if tele_on:
#                             #print(tele_on)
                            
#                         if initial_slave_time is None:
#                             initial_slave_time=timestamp.to_sec()
#                         slave_timestamps.append(timestamp.to_sec()-initial_slave_time)
                        
#                         slave_start_time = timestamp.to_sec()
#                         if initial_slave_x is None:
#                             initial_slave_x = x  # Store the initial "slave" x-coordinate
#                             initial_slave_y = y
#                             initial_slave_z = z
                
#                         slave_positions.append([ x*1000- initial_slave_x*1000, y*1000-initial_slave_y*1000,z*1000-initial_slave_z*1000])#
      

#         # Calculate distances, velocities, and accelerations
#         prev_velocity = 0
#         for i in range(1, len(slave_positions)):
#             distance = calculate_distance(slave_positions[i - 1], slave_positions[i])
#             time = slave_timestamps[i] - slave_timestamps[i - 1]  # time in seconds
#             velocity = calculate_velocity(distance, time)
#             acceleration = calculate_acceleration(velocity, prev_velocity, time)
            
#             slave_distances.append(distance)
#             slave_velocities.append(velocity)
#             slave_accelerations.append(acceleration)
            
#             prev_velocity = velocity

#         # Calculate total slave distance
#         total_slave_distance = sum(slave_distances)
#         total_velocity = sum(slave_velocities) / len(slave_velocities) if slave_velocities else 0
#         total_acceleration = sum(slave_accelerations) / len(slave_accelerations) if slave_accelerations else 0

#         print(f"Total distance traveled by slave for {bag_file}: {total_slave_distance} mm")
#         print(f"Average velocity for {bag_file}: {total_velocity} mm/s")
#         print(f"Average acceleration for {bag_file}: {total_acceleration} mm/s²")

#         # Append to results
#         results.append({
#             "File": os.path.basename(bag_file),
#             "Total_Distance_mm": total_slave_distance,
#             "Average_Velocity_mm/s": total_velocity,
#             "Average_Acceleration_mm/s2": total_acceleration
#         })

#         # Create a 3D plot for the slave trajectory
#         slave_positions_x = [pos[0] for pos in slave_positions]
#         slave_positions_y = [pos[1] for pos in slave_positions]
#         slave_positions_z = [pos[2] for pos in slave_positions]

#         fig = plt.figure()
#         ax = fig.add_subplot(111, projection='3d')
#         ax.plot(slave_positions_x, slave_positions_y, slave_positions_z, color="blue", label="Slave Trajectory")
#         ax.set_title(f"3D Trajectory - Task {a}, Attempt {b}")
#         ax.set_xlabel("X Position (mm)")
#         ax.set_ylabel("Y Position (mm)")
#         ax.set_zlabel("Z Position (mm)")
#         ax.legend()
#         plt.savefig(f"{plot_folder}/Task_{a}_Attempt_{b}.png")
#         plt.close()

# # Save results to a single CSV file
# df = pd.DataFrame(results)
# df.to_csv(output_csv_file, index=False)
# print(f"Results saved to {output_csv_file}")

