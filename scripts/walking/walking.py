#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

# generating walking pattern for Thmos
# ver: 20230805

import math
import numpy as np
from thmos_kinematics import *
from foot_step_planner import *
from preview_control import *
# import rospy
# from thmos_zmp_walk.msg import mos_pos

class walking():
  def __init__(self, **WalkParams):

    # load params
    for key, value in WalkParams.items():
        setattr(self, key, value)

    # init packet
    self.kine = THMOSLegIK(self.way_left, self.way_right, self.leg_rod_length,self.motor_offset_left, self.motor_offset_right)
    self.pc = preview_control(self.dt, self.optimize_period, self.com_height)
    self.fsp = foot_step_planner(self.max_vx, self.max_vy, self.max_vth, self.walking_period, self.foot_width, self.plan_step_num)

    # init status 
    self.X = np.matrix([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
    self.left_up = self.right_up = 0.0
    self.th = 0
    self.foot_h = self.foot_height
    self.old_vel = [0, 0, 0]
    self.both_foot_support_frames = self.both_foot_support_time / self.dt
    self.one_step_frames = self.walking_period / self.dt

    # leg offset
    self.left_off,  self.left_off_g,  self.left_off_d  = np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]) 
    self.right_off, self.right_off_g, self.right_off_d = np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]]),  np.matrix([[0.0, 0.0, 0.0]])

    # min foot step len
    self.min_foot_step_len = math.ceil(self.optimize_period / self.walking_period) + 2
    
    # step status
    self.foot_step = []
    self.foot_step_old = []
    self.foot_step_now = []
    self.foot_step_new = []
    
    # zmp status
    self.zmp_step = []
    
    # pattern
    self.pattern = []
    self.pattern_old = []
    self.pattern_now = []
    self.pattern_new_old = []
    self.pattern_new_now = []

    # create publisher topic: mos_pos, type: thmos_zmp_walk::mos_pos, size: 10
    # self.mos_publisher=rospy.Publisher('/mos_motion_pos',mos_pos,queue_size=10)
    
    self.setGoalVel()
    return

  def pos_publisher(self,current_pos):
      mos_pos_msg=mos_pos()
      mos_pos_msg.current_x=current_pos[0]
      mos_pos_msg.current_y=current_pos[1]
      mos_pos_msg.current_th=current_pos[2]
      # msg publish
      self.mos_publisher.publish(mos_pos_msg)
      rospy.loginfo("Publish mos_pos[%f,%f,%f]",
                      mos_pos_msg.current_x,mos_pos_msg.current_y,mos_pos_msg.current_th)

  def setGoalVel(self, vel = None):
    '''get vel command and be a state machine'''
    
    if ((vel == None or (self.old_vel[0] == vel[0] and self.old_vel[1] == vel[1] and self.old_vel[2] == vel[2])) and len(self.foot_step) > self.min_foot_step_len):
      # command not change
      del self.foot_step[0]
      del self.zmp_step [0]
    else:
      if (vel != None):
        self.old_vel = vel.copy()
        
      if len(self.foot_step) > 2:
        # normal status
        self.foot_step, self.zmp_step = self.fsp.step_plan(self.old_vel, self.foot_step[0])
        del self.foot_step[0]
        del self.zmp_step [0]
        
      else:
        # start status
        self.foot_step, self.zmp_step = self.fsp.step_plan(self.old_vel)
    
    current_x, current_y, current_th = self.foot_step[0][1], self.foot_step[0][2], self.foot_step[0][3]
    # self.pos_publisher([current_x,current_y,current_th])

    # caculate walk pattern from Com trajectory
    t = self.foot_step[0][0]
    self.pattern, x, y = self.pc.set_param(t, self.X[:,0], self.X[:,1], self.zmp_step)
    self.X = np.matrix([[x[0,0], y[0,0]], [x[1,0], y[1,0]], [x[2,0], y[2,0]]])
    self.foot_step_new_old, self.foot_step_new_now = self.foot_step[0].copy(), self.foot_step[1].copy()
    self.pattern_now = self.pattern.copy()
    return 
    
  def updateWalkGoal(self):
    '''get now walking goal'''
    
    # new params
    self.foot_step_old = self.foot_step_new_old.copy()
    self.foot_step_now = self.foot_step_new_now.copy()
    self.pattern_old = self.pattern_now.copy()
    
    self.left_off_g  = np.matrix([[self.foot_step_now[5], self.foot_step_now[6], self.foot_step_now[3]]])
    self.right_off_g = np.matrix([[self.foot_step_now[7], self.foot_step_now[8], self.foot_step_now[3]]])
    self.left_off_d  = (self.left_off_g - self.left_off) / (self.one_step_frames - self.both_foot_support_frames)
    self.right_off_d = (self.right_off_g - self.right_off) / (self.one_step_frames - self.both_foot_support_frames)
      
      
  def getNextPos(self):
    '''set each movement'''
    # type keep
    self.left_off = self.left_off.astype(np.float64)
    self.left_off_d = self.left_off_d.astype(np.float64)
    self.right_off = self.right_off.astype(np.float64)
    self.right_off_d = self.right_off_d.astype(np.float64)

    # init
    X = self.pattern_old.pop(0)
    
    # period
    period = round(self.one_step_frames)
    start_up = round(self.both_foot_support_frames)
    end_up   = period / 2
    period_up = end_up - start_up
    # make drop slowly
    period_do = period - end_up - start_up
    
    # move foot in the axes of x,y,theta
    if (period-len(self.pattern_old)) > start_up:
       self.right_off += self.right_off_d
       self.left_off  += self.left_off_d
       if (period-len(self.pattern)) > (start_up + period_up * 2):
         self.right_off = self.right_off_g.copy()
         self.left_off  = self.left_off_g.copy()
         
    # up and drop foot      
    if self.foot_step_now[4] == 'left':
      # up or down foot
      if start_up < (period-len(self.pattern_old)) <= end_up:
        self.left_up  += self.foot_h/period_up
      elif self.left_up > 0:
        self.left_up  = max(self.left_up  - self.foot_h/period_do, 0.0)
    elif self.foot_step_now[4] == 'right':
      if start_up < (period-len(self.pattern_old)) <= end_up:
        self.right_up += self.foot_h/period_up
      elif self.right_up > 0:
        self.right_up = max(self.right_up - self.foot_h/period_do, 0.0)
    else:
      pass
        
    # foot theta
    self.th = self.foot_step_now[3]
    
    # caculate foot pos
    lo = self.left_off  - np.block([[X[0,0:2],0]])
    ro = self.right_off - np.block([[X[0,0:2],0]])
    
    left_foot  = [lo[0,0] + self.com_x_offset, lo[0,1] + self.com_y_offset + self.ex_foot_width, self.left_up * self.foot_step_now[9]-  self.full_leg_length, 0.0, 0.0, self.th-lo[0,2]]
    right_foot = [ro[0,0] + self.com_x_offset, ro[0,1] + self.com_y_offset - self.ex_foot_width, self.right_up * self.foot_step_now[9]- self.full_leg_length, 0.0, 0.0, self.th-ro[0,2]]

    l_joint_angles = self.kine.LegIKMove('left',left_foot)
    r_joint_angles = self.kine.LegIKMove('right',right_foot)
    self.joint_angles = r_joint_angles + l_joint_angles  # R angle first

    xp = [X[0,2], X[0,3]]

    return self.joint_angles, left_foot, right_foot, xp, len(self.pattern_old)

if __name__ == '__main__':
    pass
