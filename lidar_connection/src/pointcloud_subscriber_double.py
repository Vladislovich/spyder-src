#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import pcl
import os
from std_msgs.msg import Int32  # Import Int32 for the flag message type

# Get the path to the current directory
current_dir = os.path.dirname(os.path.realpath(__file__))
pcd_file_path = os.path.join(current_dir, "pointcloud_data.pcd")

# Flag to control when to save the PointCloud
activate_flag = False

def cloud_callback(cloud):
    global activate_flag
    if not activate_flag:
        rospy.loginfo("PointCloud not saved because the flag is not activated.")
        return
    
    # Convert PointCloud2 to a list of points
    points = list(pc2.read_points(cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True))

    rospy.loginfo(f"-----------Received PointCloud with {len(points)} points-----------")

    # Convert to PCL PointCloud object
    pcl_data = pcl.PointCloud_PointXYZI()
    pcl_points = []
    for point in points:
        x, y, z, intensity = point
        pcl_points.append([x, y, z, intensity])
        rospy.loginfo(f"{x:.4f} {y:.4f} {z:.4f} {intensity:.0f}\n")
    
    # Convert the list to a PCL-compatible format and save to .pcd file
    pcl_data.from_list(pcl_points)
    pcl.save(pcl_data, pcd_file_path)
    rospy.loginfo(f"Data saved to {pcd_file_path}")

def flag_callback(msg):
    global activate_flag
    if msg.data == 1:
        rospy.loginfo("Activate flag received. Now saving PointCloud data.")
        activate_flag = True
    else:
        rospy.loginfo("Deactivate flag received. PointCloud data will not be saved.")
        activate_flag = False

def main():
    rospy.init_node('pointcloud2_to_pcd', anonymous=True)
    
    # Subscriber for the PointCloud2 data
    rospy.Subscriber("/velodyne_points", PointCloud2, cloud_callback)
    
    # Subscriber for the activate flag
    rospy.Subscriber("/activate_flag", Int32, flag_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
