#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
# foot step planner
# ver: 20230918

class foot_step_planner():
  def __init__(self, max_stride_x, max_stride_y, max_stride_th, period, width, plan_step_num = 20):
  
    # max speed of step 
    self.max_stride_x = max_stride_x
    self.max_stride_y = max_stride_y
    self.max_stride_th = max_stride_th
    
    # step num
    self.bot_step_num = plan_step_num

    # record last command
    self.goal_vx_r = 0
    self.goal_vy_r = 0
    self.goal_th_r = 0
    
    # half T
    self.period = period
    
    # lenth between legs
    self.width = width
    
    # start state
    self.start_step = [0, 0, 0, 0, 'both', 0, self.width , 0, -self.width, 0]

  def step_plan(self, goal_v, now_step = None):
    """
    Parameters;
     goal_v - [goal_vx, goal_vy, goal_vth]
     now_step - [time, (com_step)x, y, theta, support_leg, (left_foot_step)x, y, (right_foot_step)x, y, step_k]
    Returns:
     step_list - [now_step, next_step,..., last_step]
     target_zmp_list - [[t0,px,py],[t1,px,py],...]
    """ 
    
    # limit speed
    goal_vx = max(-self.max_stride_x, min(goal_v[0], self.max_stride_x))
    goal_vy = max(-self.max_stride_y, min(goal_v[1], self.max_stride_y))
    goal_vth = max(-self.max_stride_th, min(goal_v[2], self.max_stride_th))  
    
    print('new command, speed after limit: ' + str([goal_vx, goal_vy, goal_vth])) 

    # strides in each direction
    stride_x  = goal_vx 
    stride_y  = goal_vy 
    stride_th = goal_vth
    
    time = 0.
    step_list = []
    start_Flag = False
    
    # now state
    if (now_step == None):
      now_step = self.start_step.copy()
      start_Flag = True
      
    now_x  = now_step[1]
    now_y  = now_step[2]
    now_th = now_step[3]
    now_support_leg = now_step[4]
    now_l_x = now_step[5]
    now_l_y = now_step[6]
    now_r_x = now_step[7]
    now_r_y = now_step[8]
    now_s_k = now_step[9]
    
    k_stop = 0
    
    for i in range(self.bot_step_num):
      # update step list
      step_list += [[time, now_x, now_y, now_th, now_support_leg, now_l_x, now_l_y, now_r_x, now_r_y, now_s_k]]
           
      # update support leg        
      if(goal_vx == 0 and goal_vy == 0 and goal_vth == 0):
        # v == 0
        if (k_stop < 2 and start_Flag!=True and (self.goal_vx_r != 0 or self.goal_vy_r != 0 or self.goal_th_r != 0)):
          # step twice to stand
          if now_support_leg == 'left':
            now_support_leg = 'right'
          else:
            now_support_leg = 'left'  
          k_stop = k_stop + 1
        else:
          now_support_leg = 'both'
      else:
        # v!= 0
        if now_support_leg == 'left':
          now_support_leg = 'right'
        else:
          # left step first 
          now_support_leg = 'left'               
      
      # update com pos
      now_x  = now_x  + stride_x
      if((now_support_leg == 'right' and goal_vy < 0) or (now_support_leg == 'left' and goal_vy > 0)):
        now_y  = now_y  + stride_y
      else:
        now_y  = now_y
      now_th = now_th + stride_th
      
      # update foot pos
      if now_support_leg == 'left':
         now_l_x = now_x
         now_l_y = now_y + self.width 
      elif now_support_leg == 'right':
         now_r_x = now_x
         now_r_y = now_y - self.width  
      else:
         now_r_x = now_x
         now_r_y = now_y - self.width 
         now_l_x = now_x
         now_l_y = now_y + self.width                
         
      # first step modify
      if((goal_vx != 0 or goal_vy != 0 or goal_vth != 0) and (self.goal_vx_r == 0 and self.goal_vy_r == 0 and self.goal_th_r == 0)):
        now_s_k = i * 0.1
      elif(now_s_k < 1):
        now_s_k += 0.1 
      if(now_s_k >= 1):
        now_s_k = 1 
      
      # update time
      time += self.period
      
      # record old speed
      self.goal_vx_r = goal_vx
      self.goal_vy_r = goal_vy
      self.goal_th_r = goal_vth
      
    
    # zmp plan
    target_zmp_list = self.target_zmp_plan(step_list)
    return step_list, target_zmp_list


  def target_zmp_plan(self, step_plan):  
    tzmp_list = []
    for i in range(self.bot_step_num):
      now_support_leg = step_plan[i][4]
      now_step = step_plan[i]
      if now_support_leg == 'left':
         tzmp_list += [[now_step[0],now_step[5], now_step[6]]]
      elif now_support_leg == 'right':
         tzmp_list += [[now_step[0],now_step[7], now_step[8]]]
      else:
         tzmp_list += [[now_step[0],now_step[1], now_step[2]]]
    return tzmp_list
    
if __name__ == '__main__':
  planner = foot_step_planner(0.06, 0.04, 0.1, 0.4, 0.04)
  step_plan, tzmp_plan = planner.step_plan([0.01, 0.01, 0.01])
  for i in tzmp_plan:
    print(i)
