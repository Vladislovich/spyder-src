#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import serial

def main():
    # Инициализация ROS-ноды
    rospy.init_node('arduino_listener', anonymous=True)

    # Публишер для топика "/arduino_angle"
    publisher = rospy.Publisher('/arduino_angle', String, queue_size=10)

    # Подключение к Arduino через Serial порт
    port_name = '/dev/arduino'  # Замените на ваш порт
    baud_rate = 115200

    try:
        ser = serial.Serial(port_name, baud_rate, timeout=1)
        rospy.loginfo(f"Connected to Arduino on {port_name}")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to connect to Arduino: {e}")
        return

    try:
        while not rospy.is_shutdown():
            # Чтение данных от Arduino
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    rospy.loginfo(f"Received: {data}")
                    publisher.publish(data)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        ser.close()
        rospy.loginfo("Disconnected from Arduino")

if __name__ == '__main__':
    main()
