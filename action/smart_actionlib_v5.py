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
import ikfastpy  
# now we can import mod
import transformation
 




JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    
client = None

# Initialize kinematics for UR5 robot arm
ur3_kin = ikfastpy.PyKinematics()
n_joints = ur3_kin.getDOF()






posi_esfera = Pose()
pose_msg = Pose()
pos_puntero= PoseStamped()
pose_terminal= Pose()
locate_robot = Pose()
#posi_terminal = Pose()
posi_deseada = Pose()
pos_pinza = Pose()
ban_ac_tra=0
        

xx=0
yy=0
zz=0

rx1=0
ry1=0
rz1=0

pose_msg.orientation.x = 0
pose_msg.orientation.y = 0
pose_msg.orientation.z = 0
pose_msg.orientation.w = 0


def modelo(joint_angles,x,y,z,rx,ry,rz):
   
   # print("joint que llega:")
  #  print(np.round(joint_angles, decimals=1))   
    joint_angles = np.round(joint_angles, decimals=3) # in radians

    # Test forward kinematics: get end effector pose from joint angles
  #  print("\nTesting forward kinematics:\n")
  #  print("Joint angles:")
   # print(joint_angles)
    ee_pose = ur3_kin.forward(joint_angles)
    ee_pose = np.asarray(ee_pose).reshape(3,4) # 3x4 rigid transformation matrix
    print("\nEnd effector pose:")
    print(ee_pose)
   
   
    ee_pose[0,3]=ee_pose[0,3]+x/50
    ee_pose[1,3]=ee_pose[1,3]+y/50
    ee_pose[2,3]=ee_pose[2,3]+z/50
   
   
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






class NumberCounter:

    def __init__(self):
        self.counter = 0
        self.pos_x=0
        self.pos_y=0
        self.pos_z=0
        
        self.var_x=0
        self.var_y=0
        self.var_z=0

        self.var2_x=0
        self.var2_y=0
        self.var2_z=0

        self.var2_qx=0
        self.var2_qy=0
        self.var2_qz=0
        self.var2_qw=0
        
        self.var2_ax=0
        self.var2_ay=0
        self.var2_az=0
        self.var2_aw=0


        
        
        self.parametro_modo=0
        pos_puntero.pose.position.x=0
        pos_puntero.pose.position.y=0
        pos_puntero.pose.position.z=0
        
        
        
        self.pos_z=0
        self.boton_3=True
        self.boton_2=True
        self.boton_0=True
        self.boton_1=True
        self.boton_L2=True
        self.boton_R1=True
        self.res=False
        
     

     #en unity posesatampe subcriber es solamente pose   
           


        self.number_subscriber1 = rospy.Subscriber("/posetopic", Pose, self.callback_posetopic)
        
        

        self.number_subscriber3 = rospy.Subscriber("/smar_mode", Int8, self.callback_mode)
    
        self.number_subscriber4 = rospy.Subscriber("/joy", Joy, self.callback_joy)
    

    def callback_mode(self, msg):  #el programa de c++ recibe los botones de cambio de modo y los publica, en esta funcion se inicializa la variable paremetro de modo, para que cuando se reciba los cambios de posicion se conozca a quien pertenece
        if msg.data == 1 :
      #      print("modo 1")
            self.parametro_modo=1
        
        if msg.data == 2 :
        #    print("modo 2")
            self.parametro_modo=2
       
        if msg.data == 3:
        #    print("modo 3")
            self.parametro_modo=3

        if msg.data == 4:
        #    print("modo 3")
            self.parametro_modo=4

        if msg.data == 5:
            self.parametro_modo=5            

        if msg.data == 6:
            print("modo seguro")
            self.parametro_modo=6            
    
    
    def callback_joy(self, msg):   #recibe parametros directamete del joystic y los inicializa
   
     pose_msg = Pose()
     
     if msg.buttons[3] == True :
     
            self.boton_3=True
     else:
            self.boton_3=False
     
            
     if msg.buttons[2] == True :

            self.boton_2=True
     else:
            self.boton_2=False

     if msg.buttons[4] == True :

            self.boton_L2=True  
     else:
            self.boton_L2=False  

     if msg.buttons[5] == True :

            self.boton_R1=True  
     else:
            self.boton_R1=False  

            
  
     if msg.buttons[6] == True :

            self.boton_L1=True  
     else:
            self.boton_L1=False  
            

     if msg.buttons[1] == True :

            self.boton_1=True  
     else:
            self.boton_1=False  
 



# conversion de euler a quaternion

    def euler_to_quaternion(self,yaw, pitch, roll):

        self.var2_ax = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.var2_ay = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.var2_az = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.var2_aw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

     

#

    def callback_posetopic(self, msg):  #recibe cuando hay cambio en la posicion del joystic, en ese caso, se consulta en que modo se encuentra para estableceer un plan de trabajo respectivo a cada modo.
   

        xx=msg.position.x
        yy=msg.position.y
        zz=msg.position.z
        
        rx1=msg.orientation.x
        ry1=msg.orientation.y
        rz1=msg.orientation.z
        
        
 #       self.pb_mov_pinza.publish(pos_pinza) 
  #      if (self.res==False):
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
                Q1=modelo(joints_pos,xx,yy,zz,rx1,ry1,rz1)
              
                
                
                g.trajectory.points = [
                    JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
                    JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(0.2))]
                client.send_goal(g)
                res=client.wait_for_result()
           #     print res
            except KeyboardInterrupt:
                client.cancel_goal()
                raise
            except:
                raise

        except KeyboardInterrupt:
        
            #print 'no ejecuta'
            rospy.signal_shutdown("KeyboardInterrupt")
            raise
             


    

if __name__ == '__main__':
    rospy.init_node('mover_puntero')
    NumberCounter()
    rospy.spin()
