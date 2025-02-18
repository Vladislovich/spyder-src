#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
import os

def read_data_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file:
            values = list(map(float, line.strip().split()))
            if len(values) >= 3:  # Проверяем, что есть хотя бы 3 значения
                data.append(values[:3])  # Берем только первые три значения (x, y, z)
            else:
                rospy.logwarn(f"Пропущена строка: {line.strip()} - должно быть 3 значения")
    return np.array(data)

def create_point_cloud(points):
    # Определяем заголовок
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "velodyne"  # Убедитесь, что эта рамка существует в вашем TF

    # Создаём PointCloud2 сообщение
    point_cloud = pc2.create_cloud_xyz32(header, points)
    return point_cloud

def main():
    rospy.init_node('point_cloud_publisher', anonymous=True)
    pub = rospy.Publisher('point_cloud', PointCloud2, queue_size=10)

    current_dir = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(current_dir+"/logs", 'pointcloud_log_2.txt')
    
    points = read_data_from_file(filename)

    rate = rospy.Rate(10)  # Частота публикации
    while not rospy.is_shutdown():
        point_cloud = create_point_cloud(points)
        pub.publish(point_cloud)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
