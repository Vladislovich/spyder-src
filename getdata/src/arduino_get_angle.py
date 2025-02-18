#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
import serial

zero_ang_flag = False
zero_ang = 0

def zero_angle_callback(msg, ser):
    if msg.data == 1:
        global zero_ang_flag 
        zero_ang_flag = True

def main():
    global zero_ang_flag
    global zero_ang
    # Инициализация ROS-ноды
    rospy.init_node('arduino_listener', anonymous=True)

    # Публишер для топика "/arduino_angle"
    publisher = rospy.Publisher('/arduino_angle', Float32, queue_size=10)

    # Подключение к Arduino через Serial порт
    port_name = '/dev/arduino'  # Замените на ваш порт
    baud_rate = 115200

    try:
        ser = serial.Serial(port_name, baud_rate, timeout=1)
        rospy.loginfo(f"Connected to Arduino on {port_name}")
    except serial.SerialException as e:
        rospy.logerr(f"Failed to connect to Arduino: {e}")
        return

    # Подписчик на топик "/zero_angle"
    rospy.Subscriber('/zero_angle', Int32, zero_angle_callback, ser)

    try:
        while not rospy.is_shutdown():
            # Чтение данных от Arduino            
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                if data:
                    if zero_ang_flag == True:
                        zero_ang = float(data)
                        zero_ang_flag = False
                    val = float(data) - zero_ang
                    rospy.loginfo(f"Received: {val:.2f}")
                    publisher.publish(val)
            rospy.sleep(0.025)
    finally:
        ser.close()
        rospy.loginfo("Disconnected from Arduino")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass