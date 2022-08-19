#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Int8
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from sensor_msgs.msg import Joy
import numpy as np 

import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
# importing the sys module
#import sys         
  
# appending the directory of mod.py 
# in the sys.path list
#sys.path.append('~/Proy_UNity/ikfastpy')        
import ur3_ikfast
# now we can import mod
import transformation
 




JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    
client = None

# Initialize kinematics for UR3 robot arm
ur3_kin = ur3_ikfast.PyKinematics()
n_joints = ur3_kin.getDOF()



xx=0
yy=0
zz=0

rx1=0
ry1=0
rz1=0


var_x=0
var_y=0
var_z=0

var2_x=0
var2_y=0
var2_z=0

var2_qx=0
var2_qy=0
var2_qz=0
var2_qw=0

var2_ax=0
var2_ay=0
var2_az=0
var2_aw=0
ni=0




def modelo_traj(joint_angles,x,y,z):
   
   # print("joint que llega:")
  #  print(np.round(joint_angles, decimals=1))   
    joint_angles = np.round(joint_angles, decimals=3) # in radians

    # Test forward kinematics: get end effector pose from joint angles
  #  print("\nTesting forward kinematics:\n")
  #  print("Joint angles:")
    print(joint_angles)

    print "x , y",x,y
    
     
    joints_pos2=np.asarray(joints_pos)-[3.1416,0,0,0,0,0]
    ee_pose = ur3_kin.forward(joints_pos2)
    
    
#    ee_pose = ur3_kin.forward(joint_angles)
    ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
    print("\nEnd posicion actual:")
    print(ee_pose)
   
   
    ee_pose[0,3]=x
    ee_pose[1,3]=y
    ee_pose[2,3]=z
   
   
      # OJO LA ROTACION REQUIERE AJUSTES
    # position = np.zeros((3, 1))
    # position[0]=ee_pose[0,3]+x/20
    # position[1]=ee_pose[1,3]+y/20
    # position[2]=ee_pose[2,3]+z/20
    
    
    
  
    # tipo_rot='xyx'

    # sub_matriz=ee_pose[:,0:3]
    # angles = transformation.rotation_angles(sub_matriz,tipo_rot )
    # tx=angles[0]+rx
    # ty=angles[1]+ry
    # tz=angles[2]+rz
    
  
    
    # rotation_mat = transformation.rotation_matrix(tx, ty, tz, tipo_rot)
    
    # ee_pose = np.c_[rotation_mat, position]
    print("nueva pose:")
    print(ee_pose)
  
    print("\n-----------------------------")
    ee_pose = np.asarray(ee_pose)
  #  print(ee_pose)
    # Test inverse kinematics: get joint angles from end effector pose
   # print("\nTesting inverse kinematics:\n")
    joint_configs = ur3_kin.inverse(ee_pose.reshape(-1).tolist())
    n_solutions = int(len(joint_configs)/n_joints)
   # print("%d solutions found:"%(n_solutions))
    joint_configs = np.asarray(joint_configs).reshape(n_solutions,n_joints)

    index_ar=[]
    for joint_config in joint_configs:

     #   print("hola")
        index_ar.append(np.sum(np.abs(joint_config-np.asarray(joint_angles))))
        
    #    print(joint_config)

    # Check cycle-consistency of forward and inverse kinematics
  #  assert(np.any([np.sum(np.abs(joint_config-np.asarray(joint_angles))) < 1e-4 for joint_config in joint_configs]))

    #result=np.abs(joint_config-np.asarray(joint_angles))

    index_min = min(range(len(index_ar)), key=index_ar.__getitem__)

    joint1=joint_configs[index_min]
  #  print("\nTest passed!",joint1)       
    return joint1  








# conversion de euler a quaternion

def euler_to_quaternion(yaw, pitch, roll):

    var2_ax = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    var2_ay = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    var2_az = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    var2_aw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

 

#

def calibracion_ursim(msg):  #recibe cuando hay cambio en la posicion del joystic, en ese caso, se consulta en que modo se encuentra para estableceer un plan de trabajo respectivo a cada modo.
#msg=[x,y,z,rx,ry,rz]

    xx=msg[0]
    yy=msg[1]
    zz=msg[2]
    


    
#       pb_mov_pinza.publish(pos_pinza) 
#      if (res==False):
    #print("moviendo pinza derecho")       
    global client
    try:
#             rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
       # print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
       #move1()
        global joints_pos
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        Q1 = [3,3,0,0,0,0]

        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            Q1=modelo_traj(joints_pos,xx,yy,zz)
            print Q1
            
            
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(0.2))]
            client.send_goal(g)
            res=client.wait_for_result()
            print g
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        except:
            raise

    except KeyboardInterrupt:
    
        #print 'no ejecuta'
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
         

#

def callback_posetopic(msg):  #recibe cuando hay cambio en la posicion del joystic, en ese caso, se consulta en que modo se encuentra para estableceer un plan de trabajo respectivo a cada modo.
#msg=[x,y,z,rx,ry,rz]

    xx=msg[0]
    yy=msg[1]
    zz=msg[2]
    


    
#       pb_mov_pinza.publish(pos_pinza) 
#      if (res==False):
    #print("moviendo pinza derecho")       
    global client
    try:
#             rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
       # print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
       #move1()
        global joints_pos
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = JOINT_NAMES
        Q1 = [3,3,0,0,0,0]

        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
            Q1=modelo_traj(joints_pos,xx,yy,zz)
            print Q1
            
            
            g.trajectory.points = [
                JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(0.2))]
            client.send_goal(g)
            res=client.wait_for_result()
            print g
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        except:
            raise

    except KeyboardInterrupt:
    
        #print 'no ejecuta'
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
         


def tray_circular(msg2):
    
   
    
    num_segmentos = msg2
    rad = 0.05
# posicion del centro del circulo
    cx = -0.28305498
    cy = -0.07585936
    # la posicion en z hay que colocarlo
    cz = 0.15730512
    pz=np.ones(msg2)
    z=pz*cz
    
    angulo = np.linspace(0, 2*np.pi, num_segmentos+1)
    x = rad * np.cos(angulo) + cx
    y = rad * np.sin(angulo) + cy
    
    tra=[x,y,z]
    
    return tra  




def tray_circular_v2(msg2):
    
# esta version consulta pa posicion actual para hacer un circulo desde alli

    try:
#             rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
       # print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
       #move1()
        global joints_pos

        Q1 = [3,3,0,0,0,0]

        try:
            joint_states = rospy.wait_for_message("joint_states", JointState)
            joints_pos = joint_states.position
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
        except:
            raise

    except KeyboardInterrupt:
    
        #print 'no ejecuta'
        rospy.signal_shutdown("KeyboardInterrupt")
        raise   

    joints_pos2=np.asarray(joints_pos)-[np.pi,0,0,0,0,0]
#    joints_pos2=np.asarray(joints_pos)+[0,0,0,0,0,0]

    print("real articulaciones")
    print(np.rad2deg(joints_pos)) 
    
    print("articulaciones actuales")
    print(np.rad2deg(joints_pos2))    
    
    ee_pose = ur3_kin.forward(joints_pos2)
    ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
    print("\n posi actual para tray")
    print(ee_pose)
   
    cx=ee_pose[0,3]
    cy=ee_pose[1,3]
    cz=ee_pose[2,3]
    
    print("\n posi actual x y z",cx,cy,cz)

    
    num_segmentos = msg2
    rad = 0.05
# posicion del centro del circulo
  #  cx = -0.28305498
  #  cy = -0.07585936
    # la posicion en z hay que colocarlo
  #  cz = 0.15730512
    pz=np.ones(msg2)
    z=pz*cz
    
    angulo = np.linspace(0, 2*np.pi, num_segmentos+1)
    x = rad * np.cos(angulo) + cx
    y = rad * np.sin(angulo) + cy
    
    tra=[x,y,z]
    
    return tra  












   
def main():
    global client
    
    #programa
    try:
  
        aa=1     

        print "hola listo ..."
        path=tray_circular_v2(10)
        path_ar = np.array(path)
        #print "rta", path_ar
        
        print "pate"
        msg2=[[path_ar[0][3], path_ar[1][3], path_ar[2][3]],[path_ar[0][4], path_ar[1][4], path_ar[2][4]],[path_ar[0][5], path_ar[1][5], path_ar[2][5]]]
    #    print msg2
        
    
        vector=[1,2,3]
        # aqui queda la programacion al enviar
        # primero hay que hacer que el valor de la maquina recibido sea el mismo
        # segundo hay que garantizar que el robot no se devuelva
        calibracion_ursim()
        # for n1 in path_ar:
            
          # #  callback_posetopic(path_ar[ni])
            # time.sleep(2)
        
    #programa        
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__':
    rospy.init_node('trayectoria')
    main()
  #  NumberCounter()
    rospy.spin()
