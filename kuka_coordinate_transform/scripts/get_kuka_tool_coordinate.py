import socket
import xml.dom.minidom
import rospy
from kuka_coordinate_transform.msg import toolpose
from image_segment.msg import target
import multiprocessing
import random

ox = 484.675568
oy = -225.562668
oz = 294.332642
oA = 13.526774
oB = -4.027119
oC = 179.305023


KUKA_IP = '192.168.1.234'
KUKA_PORT = 54600

flag = False

X = multiprocessing.Value("d", 0.0)
Y = multiprocessing.Value("d", 0.0)
Z = multiprocessing.Value("d", 0.0)
angle = multiprocessing.Value("d", 0.0)

get_pos = multiprocessing.Value("b", False)
no_grisping = True


def callback(data):
    X.value = data.x
    Y.value = data.y
    Z.value = data.z
    if data.angle > 85.0:
        angle.value = 0.0
    else:
        angle.value = data.angle
    get_pos.value = True
    # print("callback: " + str(get_pos.value))


if __name__ == '__main__':

    rospy.init_node("kuka_tool_coordinate", anonymous=True)
    pub = rospy.Publisher("kuka_tool_coordinate", toolpose, queue_size=10)
    sub = rospy.Subscriber("/target_position_in_kuka_frame", target, callback)

    # create a socket of a client
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # connect the kuka server
    connected = False
    while not connected:
        try:
            client.connect((KUKA_IP, KUKA_PORT))
            connected = True
        except socket.error as e:
            print(e)
            # pass

    rate = rospy.Rate(0.1)
    tool_pose = toolpose()
    first = True
    while not rospy.is_shutdown():
        # print("while: " + str(X.value))
        recvmsg = client.recv(1024)
        print("received")
        with open('recvmsg.xml', 'w') as f:
            f.write(recvmsg)
        xml_recvmsg = xml.dom.minidom.parseString(recvmsg)
        ToolState = xml_recvmsg.documentElement
        Pose = ToolState.getElementsByTagName('Pose')[0]

        tool_pose.X = float(Pose.getAttribute('X'))
        tool_pose.Y = float(Pose.getAttribute('Y'))
        tool_pose.Z = float(Pose.getAttribute('Z'))
        tool_pose.A = float(Pose.getAttribute('A'))
        tool_pose.B = float(Pose.getAttribute('B'))
        tool_pose.C = float(Pose.getAttribute('C'))
        # no_grisping = bool(Pose.getAttribute('G'))

        if get_pos.value and no_grisping:   # grisp
            no_grisping = False
            print("in_grisping")
            # first = False
            Pose.setAttribute('X', str(X.value + 40.0)[0:5])
            # Pose.setAttribute('X', "615.8")
            # Pose.setAttribute('Y', "73.89")
            # Pose.setAttribute('Z', "292.1")

            # 613.4-30.2284.0
            par_y = (Y.value - 26.31) * 3.0 / 200.0
            par_z = (Y.value - 23.00) * 9.0 / 100
            Pose.setAttribute('Y', str(Y.value - 10.0 + par_y)[0:5])
            Pose.setAttribute('Z', str(Z.value+55.0 - par_z)[0:5])
            Pose.setAttribute('A', str(-70-angle.value)[0:5])

            print("send to kuka_robot: " + str(X.value)
                  [0:5] + str(Y.value)[0:5] + str(Z.value)[0:5])
            get_pos.value = False

            # Pose.setAttribute('A', '-89.627510')
            # Pose.setAttribute('B', '15.340816')
            # Pose.setAttribute('C', '-86.573700')
            # if flag:
            #     Pose.setAttribute('X', '685.463562')
            #     Pose.setAttribute('Y', '0.173981')
            #     Pose.setAttribute('Z', '330.280823')
            #     # Pose.setAttribute('A', '-89.627510')
            #     # Pose.setAttribute('B', '15.340816')
            #     # Pose.setAttribute('C', '-86.573700')
            #     flag = False
            # else:
            #     Pose.setAttribute('X', '400.928772')
            #     Pose.setAttribute('Y', '100.539459')
            #     Pose.setAttribute('Z', '350.280823')
            #     # Pose.setAttribute('A', '94.983467')
            #     # Pose.setAttribute('B', '12.781209')
            #     # Pose.setAttribute('C', '91.591270')
            #     flag = True

            Pose.setAttribute('R', '1')
            Pose.setAttribute('G', '1')
            xml_recvmsg.toxml(encoding='utf8')
            client.send(str(xml_recvmsg.toxml(encoding='utf8')))

        elif not no_grisping:
            no_grisping = True
            print("in_laying")
            Pose.setAttribute('X', str(random.uniform(480, 600))[0:5])
            Pose.setAttribute('Y', str(random.uniform(-250, 300))[0:5])
            Pose.setAttribute('Z', '315.0')
            Pose.setAttribute('A', str(random.uniform(-90, 90))[0:5])
            Pose.setAttribute('R', '1')
            Pose.setAttribute('G', '0')
            xml_recvmsg.toxml(encoding='utf8')
            client.send(str(xml_recvmsg.toxml(encoding='utf8')))


        try:
            pub.publish(tool_pose)
            rate.sleep()
        except rospy.ROSInterruptException as e:
            print(e.message)

    rospy.spin()

    client.close()
