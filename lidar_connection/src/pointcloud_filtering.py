#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
import pcl
import pcl_ros

class PointCloudFilter:
    def __init__(self):
        rospy.init_node('pointcloud_filter_node', anonymous=True)
        
        # Подписка на исходный топик с данными PointCloud2
        self.pointcloud_sub = rospy.Subscriber("/point_cloud", PointCloud2, self.pointcloud_callback)
        
        # Публикация отфильтрованных данных PointCloud2
        self.pointcloud_pub = rospy.Publisher("/filtered_pointcloud", PointCloud2, queue_size=1)

    def pointcloud_callback(self, msg):
        # Конвертация PointCloud2 в массив numpy
        cloud_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        # Создание объекта PointCloud из pcl
        cloud = pc2.PointCloud2()
        cloud.from_list(cloud_points)

        # Фильтр по диапазону (например, оставить только точки, у которых координаты x в пределах от -1 до 1)
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("x")
        passthrough.set_filter_limits(-1.0, 1.0)
        
        filtered_cloud = passthrough.filter()

        # Конвертация обратно в PointCloud2 и публикация
        filtered_msg = self.convert_to_pointcloud2(filtered_cloud, msg.header)
        self.pointcloud_pub.publish(filtered_msg)

    def convert_to_pointcloud2(self, cloud, header):
        # Преобразование массива pcl обратно в сообщение PointCloud2
        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
        ]

        return pc2.create_cloud(header, fields, cloud.to_list())

if __name__ == '__main__':
    try:
        PointCloudFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
