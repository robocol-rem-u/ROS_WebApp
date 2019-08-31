#!/usr/bin/env python
from master_msgs.msg import arm_Orders
import rospy

c = 0
msg = ''
def mover():
    global c,msg
    print('mover')
    if c == 0:
        msg.message = 'F50#'
        print('up')
        c = 1
    elif c == 1:
        msg.message = 'F0#'
        print('down')
        c = 2
    elif c == 2:
        msg.message = 'F50!'
        print('down')
        c = 3
    elif c == 3:
        msg.message = 'F0#'
        print('down')
        c = 0
#    else:
#        c = 0


def prueba():
    global msg
    rospy.init_node('nodo_prueba_brazo', anonymous=True)
    pub_Arm_Orders = rospy.Publisher('topic_arm_orders',arm_Orders,queue_size=10)
    msg = arm_Orders()
    rate = rospy.Rate(1)
    print('inicio')
    while not rospy.is_shutdown():
        mover()
        pub_Arm_Orders.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        prueba()
    except rospy.ROSInterruptException:
        pass
