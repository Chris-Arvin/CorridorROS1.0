#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pygame
import rospy
import math
from geometry_msgs.msg import Twist
from pedsim_msgs.msg import AgentStates
import robot_teleop

class Person:
    def __init__(self, win_obj, id):
        self.id = id
        self.win_obj = win_obj
        self.key_pressed = [0,0,0,0]    # [move,move,gaze,gaze]

    def set_robot_teleop(self,max_lin_vel, max_ang_vel):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.robot_tel = robot_teleop.Robot(max_lin_vel, max_ang_vel)

    def draw_obj(self, location):
        (row, col) = (location[0]*WIN_OBJ_H, location[1]*WIN_OBJ_W)
        pygame.draw.lines(self.win_obj,(128,138,135),True,((col+2,row+2),(col+198,row+2),(col+198,row+198),(col+2,row+198)),1)
        position_rec = [(col+75,row+45),(col+75,row+145),(col+25,row+95),(col+125,row+95),((col+75,row+95))]
        font_key = pygame.font.SysFont('calibri',40)    ##(.ttf)
        font_id = pygame.font.SysFont('calibri',20)
        for i in range(5):
            key = font_key.render(self.keyboard[i],True,(41,36,33))
            pygame.draw.rect(self.win_obj,(220,220,220),(position_rec[i][0],position_rec[i][1],50,50))
            self.win_obj.blit(key,(position_rec[i][0]+15,position_rec[i][1]+8))
        if self.id == 'robot':
            id = font_id.render('ID: robot',True,(41,36,33))
            self.win_obj.blit(id,(position_rec[0][0]-13,position_rec[0][1]-30))
        else:
            id = font_id.render('ID: {}'.format(self.id),True,(41,36,33))
            self.win_obj.blit(id,(position_rec[0][0],position_rec[0][1]-30))
        pygame.display.update()

    def draw_gaze(self, location):
        (row, col) = (location[0]*WIN_OBJ_H, location[1]*WIN_OBJ_W)
        pygame.draw.lines(self.win_obj,(128,138,135),True,((col+2,row+2),(col+198,row+2),(col+198,row+198),(col+2,row+198)),1)
        position_rec = [(col+75,row+45),(col+75,row+145),(col+25,row+95),(col+125,row+95),((col+75,row+95))]
        font_key = pygame.font.SysFont('calibri',40)    ##(.ttf)
        font_id = pygame.font.SysFont('calibri',20)
        for i in range(5):
            key = font_key.render(self.gazekeyboard[i],True,(41,36,33))
            pygame.draw.rect(self.win_obj,(220,220,220),(position_rec[i][0],position_rec[i][1],50,50))
            self.win_obj.blit(key,(position_rec[i][0]+15,position_rec[i][1]+8))
            id = font_id.render('gaze control',True,(41,36,33))
            self.win_obj.blit(id,(position_rec[0][0]-25,position_rec[0][1]-30))
        pygame.display.update()
    
    def set_keyboard(self,keyboard):
        self.keyboard = keyboard
        self.ww = keyboard[0]
        self.xx = keyboard[1]
        self.aa = keyboard[2]
        self.dd = keyboard[3]
        self.ss = keyboard[4]

    def set_gazekeyboard(self,keyboard):
        self.gazekeyboard = keyboard
        self.guu=keyboard[0]
        self.gdd=keyboard[1]

        self.gll=keyboard[2]
        self.grr=keyboard[3]
        
    def set_publisher(self,topic):
        self.pub = rospy.Publisher(topic, Twist, queue_size=10)

    def set_gazepublisher(self,topic):
        self.gaze_pub=rospy.Publisher(topic,Twist,queue_size=10)

    def handle_key(self):
        key_msg = Twist()
        if self.key_pressed[0] == 0:
            key_msg.linear.x = 0
        if self.key_pressed[1] == 0:
            key_msg.angular.z = 0
        if self.key_pressed[0] == self.ww:
            key_msg.linear.x = 1
        if self.key_pressed[0] == self.xx:
            key_msg.linear.x = -1
        if self.key_pressed[0] == self.ss:
            key_msg.linear.x = -2
            key_msg.angular.z = -2
        if self.key_pressed[1] == self.aa:
            key_msg.angular.z = 1
        if self.key_pressed[1] == self.dd:
            key_msg.angular.z = -1
        return key_msg

    def handle_gaze_key(self):
        gazekey_msg=Twist()
                
        if self.key_pressed[2]==0:
            gazekey_msg.angular.y=0

        if self.key_pressed[3]==0:
            gazekey_msg.angular.z=0
        
        if self.key_pressed[2]==self.guu:
            gazekey_msg.angular.y=1

        if self.key_pressed[2]==self.gdd:
            gazekey_msg.angular.y=-1

        if self.key_pressed[3]==self.gll:
            gazekey_msg.angular.z=-1
        
        if self.key_pressed[3]==self.grr:
            gazekey_msg.angular.z=1
        # use to control the mode of the keyboard
        gazekey_msg.linear.x=0

        return gazekey_msg

    def key_down(self, key):
        if self.id == human_current_index or self.id=="robot":
            # deal with move
            if key == self.ww or key == self.ss or key == self.xx:
                self.key_pressed[0] = key
            if key == self.aa or key == self.dd:
                self.key_pressed[1] = key

        # deal with gaze
        if key == self.guu or key==self.gdd:
            self.key_pressed[2]=key
        if key == self.gll or key==self.grr:
            self.key_pressed[3]=key

    def key_up(self, key):
        # deal with move        
        if key == self.key_pressed[0]:
            self.key_pressed[0] = 0
        if key == self.key_pressed[1]:
            self.key_pressed[1] = 0 

        # deal with gaze
        if key == self.key_pressed[2]:
            self.key_pressed[2] = 0
        if key == self.key_pressed[3]:
            self.key_pressed[3] = 0


class InterfaceSystem:
    def __init__(self,n):    
        pygame.init()
        row_num = math.ceil(n/MAX_LINE_NUM)*WIN_OBJ_H +2
        col_num = min(n,MAX_LINE_NUM)*WIN_OBJ_W +2
        self.win_obj = pygame.display.set_mode((col_num,row_num))      
        pygame.display.set_caption('Interface to Manually Control the Objects')   
        self.win_obj.fill((255,255,255))   
        pygame.display.flip()
        
    def getWin(self):
        return self.win_obj


WIN_OBJ_H = 199
WIN_OBJ_W = 199
MAX_LINE_NUM = 6.0
robot_keyboard = ['w','x','a','d','s']
human_keyboard = ['t','b','f','h','g']
gaze_control = ['8','2','4','6','5']
gaze_current_index = 0
gaze_max_index = -1
gaze_plus = 0
global human_current_index
human_current_index = 0
human_max_index = -1
n=-1

if __name__=="__main__":
    rospy.init_node('interface_keyboard_publisher', anonymous=True)
    person_mode = rospy.get_param('/pedsim_simulator/person_mode')
    robot_mode = rospy.get_param('/pedsim_simulator/robot_mode')
    gaze_enable = rospy.get_param('/pedsim_simulator/enable_gaze_control')
    max_lin_vel = rospy.get_param('/pedsim_interface_teleop/teleop/max_vel_x')
    max_ang_vel = rospy.get_param('/pedsim_interface_teleop/teleop/max_vel_theta')
    # todo wait_for_message: how many people?
    sub_agents = rospy.wait_for_message("/pedsim_simulator/simulated_agents", AgentStates)
    n = len(sub_agents.agent_states)

    if gaze_enable and person_mode==2:
        gaze_plus = 1
    unit_list = []

    # 仅机器人需要手动控制
    if person_mode!=2 and robot_mode==2:
        n=0
        i=0
        env = InterfaceSystem(1+gaze_plus)
        location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
        obj = Person(env.getWin(),'robot')
        obj.set_keyboard(robot_keyboard)
        obj.set_robot_teleop(max_lin_vel, max_ang_vel)
        obj.set_gazekeyboard(gaze_control)            
        obj.draw_obj(location)
        unit_list.append(obj)
    
    # 仅行人需要手动控制
    elif person_mode==2 and robot_mode!=2:
        # initialize interface system    
        env = InterfaceSystem(1+gaze_plus)
        gaze_max_index = n
        human_max_index = n
        # initialize the objects for all persons
        i = 0
        while i<n:  
            location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
            obj = Person(env.getWin(),i)
            obj.set_keyboard(human_keyboard)
            obj.set_gazekeyboard(gaze_control)            
            obj.set_publisher("/people{}/keyboard".format(i))
            obj.set_gazepublisher("/people{}/keyboard/gaze".format(i))
            if i==0:
                obj.draw_obj(location)
            unit_list.append(obj)
            i+=1
    
    # 行人和机器人都是手动控制
    elif person_mode==2 and robot_mode==2:
        env = InterfaceSystem(2+gaze_plus)
        gaze_max_index = n
        human_max_index = n
        i = 0
        while i<n+1:
            # control robot with keyboard_teleop
            if i==0:
                location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
                obj = Person(env.getWin(),'robot')
                obj.set_keyboard(robot_keyboard)
                obj.set_gazekeyboard(gaze_control)            
                obj.set_robot_teleop(max_lin_vel, max_ang_vel)
                obj.draw_obj(location)
                unit_list.append(obj)
                i+=1
            # control persons
            else:    
                location = (math.floor(i/MAX_LINE_NUM),math.ceil(i%MAX_LINE_NUM))
                obj = Person(env.getWin(),i-1)
                obj.set_keyboard(human_keyboard)
                obj.set_gazekeyboard(gaze_control)
                obj.set_publisher("/people{}/keyboard".format(i-1))
                obj.set_gazepublisher("/people{}/keyboard/gaze".format(i-1))
                if i==1:
                    obj.draw_obj(location)
                unit_list.append(obj)
                i+=1
    
    if gaze_enable:
        loc = 0
        if robot_mode==2:
            loc += 1
        if person_mode==2:
            loc += 1
        location = (math.floor(loc/MAX_LINE_NUM),math.ceil(loc%MAX_LINE_NUM))
        obj.draw_gaze(location)

    # loop to read the keyboard events
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown():
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                exit()
            if event.type == pygame.KEYDOWN:
                key = None
                try:
                    key = chr(event.key)
                except:
                    if event.key==pygame.K_KP8:
                        key=chr(pygame.K_8)
                    elif event.key==pygame.K_KP2:
                        key=chr(pygame.K_2)
                    elif event.key==pygame.K_KP4:
                        key=chr(pygame.K_4)
                    elif event.key==pygame.K_KP6:
                        key=chr(pygame.K_6)
                    elif event.key==pygame.K_KP5:
                        key=chr(pygame.K_5)
                if key == chr(pygame.K_5):
                    gaze_current_index += 1
                    gaze_current_index = gaze_current_index%gaze_max_index
                elif key == "j":
                    human_current_index -= 1
                    human_current_index = human_current_index%human_max_index
                elif key == "k":
                    human_current_index += 1
                    human_current_index =human_current_index%human_max_index
                elif key:
                    for unit in unit_list:
                        unit.key_down(key)

            if event.type == pygame.KEYUP:
                try:
                    key = chr(event.key)
                except:
                    if event.key==pygame.K_KP8:
                        key=chr(pygame.K_8)
                    elif event.key==pygame.K_KP2:
                        key=chr(pygame.K_2)
                    elif event.key==pygame.K_KP4:
                        key=chr(pygame.K_4)
                    elif event.key==pygame.K_KP6:
                        key=chr(pygame.K_6)

                for unit in unit_list:
                    unit.key_up(key)

        for unit in unit_list:
            if unit.id == 'robot':
                unit.pub.publish(unit.robot_tel.loopHandleKeys(unit.key_pressed))
            else:
                key_msg = unit.handle_key()
                unit.pub.publish(key_msg)
                if gaze_current_index == unit.id and gaze_enable:
                    gaze_key_msg=unit.handle_gaze_key()
                    unit.gaze_pub.publish(gaze_key_msg)
        rate.sleep()
