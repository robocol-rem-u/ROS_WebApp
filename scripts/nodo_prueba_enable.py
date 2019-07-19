from master_msgs.srv import service_enable
import rospy

def nodo_prueba_enable():
    rospy.init_node('nodo_prueba_enable', anonymous=True)
    s = rospy.Service('service_enable', service_enable, handle_prueba)
    rospy.spin()
    while not rospy.is_shutdown():
        rospy.sleep(rospy.Rate(10))
        pass


def handle_prueba(param):
    print("Llego:", param.message)
    return []


if __name__ == '__main__':
    try:
        nodo_prueba_enable()
    except rospy.ROSInterruptException:
        pass