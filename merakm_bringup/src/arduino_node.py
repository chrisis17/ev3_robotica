#!/usr/bin/env python3
import sys
import math
import time
import rospy
import moteus
import asyncio
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

L = 0.33
R = 0.04

global vel_x, ang_z
vel_x = 0
ang_z = 0

def cmd_vel_cb(msg): # Tipo callback
  global vel_x, ang_z
  vel_x = msg.linear.x  
  ang_z = msg.angular.z

async def main():

  # Variable declaration
  global vel_x, ang_z

  # ROS Node definition
  rospy.init_node('cmd_robot')
  rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)
  pub_right = rospy.Publisher('w_right', Float32, queue_size=1)
  pub_left = rospy.Publisher('w_left', Float32, queue_size=1)
  rate = 50 # Minimun frecuency is 50 Hz
  print("[Odom] Node start working!")

  # Connection with the mjbot brushless motor driver
  transport = moteus.Fdcanusb()
  c1 = moteus.Controller(id = 1)
  c2 = moteus.Controller(id = 2)
  await transport.cycle([c1.make_stop(), c2.make_stop()])
  print("[Odom] Everything is stop!")
  
  fil_w_left = 0.0
  fil_w_right = 0.0
  last_fil_w_left = 0.0
  last_fil_w_right = 0.0

  try:
    while not rospy.is_shutdown():

      cmd_w_left = - vel_x/R + L*ang_z/(2*R);
      cmd_w_right = vel_x/R + L*ang_z/(2*R);
      result = await transport.cycle([c1.make_position(position=math.nan,velocity=cmd_w_right,query=True), c2.make_position(position=math.nan,velocity=cmd_w_left,query=True)])

      w_left = np.around((-1)*result[1].values[moteus.Register.VELOCITY], decimals=3)
      w_right = np.around(result[0].values[moteus.Register.VELOCITY], decimals=3)
      fil_w_left = 0.854*fil_w_left + 0.0728*w_left + 0.0728*last_fil_w_left
      fil_w_right = 0.854*fil_w_right + 0.0728*w_right + 0.0728*last_fil_w_right
      last_fil_w_left = fil_w_left
      last_fil_w_right = fil_w_right

      #pub_left.publish(result[1].values[moteus.Register.VELOCITY])
      #pub_right.publish(result[0].values[moteus.Register.VELOCITY])
      pub_left.publish(fil_w_left)
      pub_right.publish(fil_w_right)
      await asyncio.sleep(1/rate)
            
  except KeyboardInterrupt:
    await transport.cycle([c1.make_stop()])
    await transport.cycle([c2.make_stop()])
    print("The Odometry_mjbots node is close!")
    sys.exit()

if __name__ == '__main__':
    asyncio.run(main())
