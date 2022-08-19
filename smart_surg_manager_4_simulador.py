#!/usr/bin/env python

import rospy
from std_msgs.msg import Int64
from std_msgs.msg import Int8
from std_srvs.srv import SetBool
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from sensor_msgs.msg import Joy
import numpy as np 


posi_esfera = Pose()
pose_msg = Pose()
pos_puntero= PoseStamped()
pose_terminal= Pose()
locate_robot = Pose()
#posi_terminal = Pose()
posi_deseada = Pose()
pos_pinza = Pose()
ban_ac_tra=0
        



pose_msg.orientation.x = 0
pose_msg.orientation.y = 0
pose_msg.orientation.z = 0
pose_msg.orientation.w = 0


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
     

     #en unity posesatampe subcriber es solamente pose   
        


        self.pub = rospy.Publisher("/posi_esfer", Pose, queue_size=10)
        self.pub2 = rospy.Publisher("/xyz_robot", Pose, queue_size=10)
        self.pub_xyz_puntero = rospy.Publisher("/xyz_puntero_rviz", PoseStamped, queue_size=10)
        
        
        self.pb_robot_iz = rospy.Publisher("/xyz_robot_iz", Pose, queue_size=10)
        
        self.pb_mov_pinza = rospy.Publisher("/mov_pinza", Pose, queue_size=10)
        self.pb_mov_pinza_iz = rospy.Publisher("/mov_pinza_iz", Pose, queue_size=10)
        
        self.terminal = rospy.Publisher("/pos_terminal", Pose, queue_size=10)
        self.pub4 = rospy.Publisher("/array_pos_deseada", Pose, queue_size=10)
        self.pb_ac_traye = rospy.Publisher("/activar_trayectoria", Int8, queue_size=10)
        


        self.number_subscriber1 = rospy.Subscriber("/posetopic", Pose, self.callback_posetopic)
        self.number_subscriber = rospy.Subscriber("/state_esfera", PoseStamped, self.callback_state_base)
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
 


            
            
    
    def callback_state_base(self, msg):   #recibe la posicion actual del puntero, para que en el modo puntero sea sumada a la posicion que llega desde el joystick
    
        self.pos_x= msg.pose.position.x
        self.pos_y= msg.pose.position.y
        self.pos_z= msg.pose.position.z
        # print(self.pos_x,self.pos_y,self.pos_z)
        # print("stado actual")
        

# conversion de euler a quaternion

    def euler_to_quaternion(self,yaw, pitch, roll):

        self.var2_ax = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        self.var2_ay = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        self.var2_az = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        self.var2_aw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

     

#

    

    def callback_posetopic(self, msg):  #recibe cuando hay cambio en la posicion del joystic, en ese caso, se consulta en que modo se encuentra para estableceer un plan de trabajo respectivo a cada modo.
   

        
        if self.parametro_modo == 1  :  #se recibe la posicion del joystic, si se preciona el botor l2 se movera la pinza, y si se presiona el boton l1 se orientara el terminal del robot, en casos contrarios simplemente se desplaza el terminal del robot
            print("mov  robot derecho ")

            if self.boton_L2 == True :

                pos_pinza.position.x=msg.position.x
                pos_pinza.position.y=msg.position.y
                pos_pinza.position.z=msg.position.z
                self.pb_mov_pinza.publish(pos_pinza)  
                print("moviendo pinza derecho")       

            if self.boton_L1 == True :

                locate_robot.orientation.x=msg.orientation.x
                locate_robot.orientation.y=msg.orientation.y
                locate_robot.orientation.z=msg.orientation.z
                self.pub2.publish(locate_robot)  
                print("orientation robot derecho")       
 
            else :

                locate_robot.position.x=msg.position.x
                locate_robot.position.y=msg.position.y
                locate_robot.position.z=msg.position.z
                self.pub2.publish(locate_robot)  
                print("moviendo robot derecho")
   


        
        if self.parametro_modo == 2  :
            print("mov  robot_ iz")

            if self.boton_L2 == True :

                pos_pinza.position.x=msg.position.x
                pos_pinza.position.y=msg.position.y
                pos_pinza.position.z=msg.position.z
                self.pb_mov_pinza_iz.publish(pos_pinza)  
                print("moviendo pinza iz")       

            if self.boton_L1 == True :

                locate_robot.orientation.x=msg.orientation.x
                locate_robot.orientation.y=msg.orientation.y
                locate_robot.orientation.z=msg.orientation.z
                self.pb_robot_iz.publish(locate_robot)  
                print("orientation robot iz")       
 
            else :

                locate_robot.position.x=msg.position.x
                locate_robot.position.y=msg.position.y
                locate_robot.position.z=msg.position.z
                self.pb_robot_iz.publish(locate_robot)  
                print("moviendo robot iz")
   









   
        if self.parametro_modo == 3: #actualmente solo gira un objeto en cada eje.
            print(" terminal")           
            
            pose_terminal.orientation.x=msg.position.x*10
            pose_terminal.orientation.y=msg.position.y*10
            pose_terminal.orientation.z=msg.position.z*10
            pose_terminal.orientation.w=0
            pose_terminal.position.y=0
            pose_terminal.position.z=0
            pose_terminal.position.z=0
            self.terminal.publish(pose_terminal)  
            
        if self.parametro_modo == 4: # desplaza un puntero en el mundo virtual
            print("puntero")        


           
            
            self.euler_to_quaternion(msg.orientation.x/1,msg.orientation.y/1,msg.orientation.z/1)

            self.var2_qx=self.var2_qx+ self.var2_ax/100
            self.var2_qy=self.var2_qy+ self.var2_ay/100
            self.var2_qz=self.var2_qz+ self.var2_az/100
            self.var2_qw=self.var2_qw+ self.var2_aw/100
  
            self.var2_x=self.var2_x+msg.position.x/100
            self.var2_y=self.var2_y+msg.position.y/100
            self.var2_z=self.var2_z+msg.position.z/100

            posi_esfera.position.z=self.var2_z
            posi_esfera.position.x=self.var2_x
            posi_esfera.position.y=self.var2_y
  
            posi_esfera.orientation.z=self.var2_qz
            posi_esfera.orientation.x=self.var2_qx
            posi_esfera.orientation.y=self.var2_qy
            posi_esfera.orientation.w=self.var2_qw
            
            self.pub.publish(posi_esfera)  
            print("desplazando y rotando el puntero")           


  # explicar los quaterniones https://quaternions.online/

  
        
            self.var_x=self.var_x+msg.position.x/100
            self.var_y=self.var_y+msg.position.y/100
            self.var_z=self.var_z+msg.position.z/100
     


      
            pos_puntero.pose.position.x=self.var_x
            pos_puntero.pose.position.y=self.var_y
            pos_puntero.pose.position.z=self.var_z
            
        
            
            
            
            # se define el formato para publicar en el topico
            

            #posest.header.seq = 1
            pos_puntero.header.stamp = rospy.Time.now()
            
         
            pos_puntero.header.frame_id = "camera_color_frame"

            
            self.pub_xyz_puntero.publish(pos_puntero)             
            
            
            
            if self.boton_3 == True :
                posi_deseada.position.x= posi_esfera.position.x
                posi_deseada.position.y= posi_esfera.position.y
                posi_deseada.position.z= posi_esfera.position.z
                self.pub4.publish(posi_deseada)  
                print("puntero ini y boton 3")           
            
            if self.boton_2 == True :
                posi_deseada.orientation.x= posi_esfera.position.x
                posi_deseada.orientation.y= posi_esfera.position.y
                posi_deseada.orientation.z= posi_esfera.position.z
                self.pub4.publish(posi_deseada)  
                print("puntero fin y boton 2")
            aso=1

                
            
            
  
        if self.parametro_modo == 5: #modo de activacion de la trayectoria si enviar 2
            
            if self.boton_1 == True :
                ban_ac_tra=2
                self.pb_ac_traye.publish(ban_ac_tra)  
            else:
               ban_ac_tra=1
               self.pb_ac_traye.publish(ban_ac_tra)  
             
  
            print("mo trayectoria")           
 
 
 
        if self.parametro_modo == 6:
            print("mo seguro")         
    

if __name__ == '__main__':
    rospy.init_node('mover_puntero')
    NumberCounter()
    rospy.spin()
