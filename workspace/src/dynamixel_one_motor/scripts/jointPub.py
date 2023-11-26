import csv
import math
import os

import pandas as pd
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_workbench_msgs.srv import DynamixelCommand


Torques=[1023,1023,1023,1023,1023]

def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))
def callback(data):
    rospy.loginfo(data.position)
    print(data)
    if data.position[4]>-2:
        print('gripper abierto')
    else:
        print('gripper cerrado')
def leer_trayectorias():
    global dataT
    package_dir = os.path.dirname(os.path.abspath(__file__))
    ruta_archivo_csv= os.path.join(package_dir, "figuras/q.csv")
    # Utiliza la función read_csv para cargar los datos en un DataFrame
    dataT = pd.read_csv(ruta_archivo_csv, header=None,sep=',')
    dataT = (dataT.applymap(replace_commas))
    print(len(dataT.values))


def replace_commas(value):
    # Verificar si el valor es una cadena y contiene una coma
    if isinstance(value, str):
        # Reemplazar la coma por un punto y convertir a float
        return float(value.replace(',', '.'))
    else:
        # Devolver el valor sin cambios si no es necesario reemplazar
        return (value)

    
def listener():
    #rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, callback)
    #rospy.spin()
def ajus_torque(T):
    #Definir los límites de torque de los motores.
    for i in range(5):    
        jointCommand('', (i+1), 'Torque_Limit', T[i], 0)   
def joint_publisher(q,td,ts):
    pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=0)

    #while not rospy.is_shutdown():
    state = JointTrajectory()
    state.header.stamp = rospy.Time.now()
    state.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
    point = JointTrajectoryPoint()
    point.positions = [q[0], q[1],q[2],q[3],q[4]]    
    point.time_from_start = rospy.Duration(td)
    state.points.append(point)
    pub.publish(state)
    print('published command')
    rospy.sleep(ts)


if __name__ == '__main__':
    try:
        while True:
            rospy.init_node('joint_publisher', anonymous=False)
            listener()
            ajus_torque(Torques)
            joint_publisher([math.radians(90),math.radians(0),math.radians(0),math.radians(-90),math.radians(0)],1.5,2)
            rospy.sleep(3)
            joint_publisher([math.radians(90),math.radians(0),math.radians(0),math.radians(-90),math.radians(53)],1.5,2)
       # leer_trayectorias()    
    except rospy.ROSInterruptException:
        pass