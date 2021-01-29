# ECE183DA Capstone - Joint Lab Assignment #1 Simulation
# Farooq Akhtar
# Hayato Kato
# Matthew Jeong

import math
import numpy as np
import matplotlib.pyplot as plt

# Global Constants
rw = 90 # Robot width (mm)
rd = 10 # Robot wheel diameter (mm)
rl = 75 # Robot length (mm)
eh = 400 # Environment height (mm)
ew = 400 # Environment width (mm)
delta_t = 1 # Time elapsed between iterations

class WheelRobot():
  def __init__(self, rw, rd, rl, x, y, theta):
    self.width = rw # Width of the robot's drivetrain
    self.diameter = rd # Diameter of the robot's wheels
    self.theta = theta # Theta of the robot
    self.length = rl # Length of the robot
    self.state = [x, y, theta] # Current state of the robot (x&y=position, theta=orientation)
    
  def move(self, wl, wr, debug=False):
    # System Dynamics
    #temp_theta = self.state[2] + self.diameter/self.width*(wr-wl)*delta_t
    #temp_x = self.state[0] + self.diameter/2*(wr+wl)*math.cos(math.degrees(temp_theta))*delta_t
    #temp_y = self.state[1] + self.diameter/2*(wr+wl)*math.sin(math.degrees(temp_theta))*delta_t
    temp_theta = self.state[2] + self.diameter/(self.width)*(wr-wl)*delta_t
    temp_x = self.state[0] + self.diameter/2*(wr+wl)*math.cos(0.5*(self.state[2]+temp_theta))*delta_t
    temp_y = self.state[1] + self.diameter/2*(wr+wl)*math.sin(0.5*(self.state[2]+temp_theta))*delta_t
    

    temp_state = [temp_x, temp_y, temp_theta]
    

    # Output Equation
    wallX = [0,ew,ew,0]
    wallY = [0,0,eh,eh]
    
    maxLength = math.sqrt(eh*eh+ew*ew)

    #print("maxlength:",maxLength)
    
    x1 = temp_state[0]
    y1 = temp_state[1]
    x2_f = x1 + maxLength*math.cos(temp_state[2])
    y2_f = y1 + maxLength*math.sin(temp_state[2])
    x2_r = x1 + maxLength*round(math.cos(temp_state[2]-math.pi/2),4)
    y2_r = y1 + maxLength*round(math.sin(temp_state[2]-math.pi/2),4)
    px_f = -1
    py_f = -1
    px_r = -1
    py_r = -1
    for i in range(4):
      x3 = wallX[i]
      y3 = wallY[i]
      j = i+1
      if j >= 4:
        j=0
      x4 = wallX[j]
      y4 = wallY[j]

      #print("Point2_f:",x2_f,y2_f)
      #print("Point2_r:",x2_r,y2_r)
      
      denominator_f = (x1-x2_f)*(y3-y4)-(y1-y2_f)*(x3-x4)
      if denominator_f == 0:
        continue
      m_f = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/denominator_f
      n_f = ((x2_f-x1)*(y1-y3)-(y2_f-y1)*(x1-x3))/denominator_f
      
      if m_f >= 0 and m_f <= 1 and n_f >= 0 and n_f <= 1:
        px_f = x1+m_f*(x2_f-x1)
        py_f = y1+m_f*(y2_f-y1)
        
      denominator_r = (x1-x2_r)*(y3-y4)-(y1-y2_r)*(x3-x4)
      if denominator_r == 0:
        continue
      m_r = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/denominator_r
      n_r = ((x2_r-x1)*(y1-y3)-(y2_r-y1)*(x1-x3))/denominator_r
      
      if m_r >= 0 and m_r <= 1 and n_r >= 0 and n_r <= 1:
        px_r = x1+m_r*(x2_r-x1)
        py_r = y1+m_r*(y2_r-y1)
      #print("--------")
      #print(denominator_r)
      #print(m_r)
      #print(n_r)
      #print("--------")
   
    #print("END OF LOOP")
    #print(px_f,py_f)
    #print(px_r,py_r)
    #print(maxLength)
    #print(x1,y1)
    #print(x2,y2)
    #print(x3,y3)
    #print(x4,y4)

    if px_f == -1 and py_f == -1:
      px_f = temp_state[0]
      py_f = 0
    if px_r == -1 and py_r == -1:
      px_r = temp_state[0]
      py_r = 0

    # Sensor Output
    D_f = math.sqrt((px_f-temp_state[0])*(px_f-temp_state[0])+(py_f-temp_state[1])*(py_f-temp_state[1]))
    D_r = math.sqrt((px_r-temp_state[0])*(px_r-temp_state[0])+(py_r-temp_state[1])*(py_r-temp_state[1]))
    Omega = temp_state[2] - self.state[2]
    B_x = -math.cos(temp_state[2])
    B_y = math.sin(temp_state[2])
    
    output = [D_f, D_r, Omega, B_x, B_y]
    
    print("Output: ",output)
    
    self.state = temp_state

    #if debug:
      #print("X pos: " + str(round(self.state[0],2)) + "\tY pos: " + str(round(self.state[1],2)) + "\tTheta: " + str(round(math.degrees(self.state[2]),2)))
    
    return (output)
    #return (px_f, py_f, px_r, py_r)
  #def draw(self):
    
    

class Enviro():
  def __init__(self, h, w):
    # Initialize class variables
    self.enviroHeight = h
    self.enviroWidth = w
    self.wheelRobot = WheelRobot(rw, rd, rl, w/2, h/2, 0)

  def draw(self, x_sightline_f, y_sightline_f, x_sightline_r, y_sightline_r, title=""):
    drawState = self.wheelRobot.state
    x_coord = [drawState[0]]
    y_coord = [drawState[1]]
    x_dir = math.cos(drawState[2])
    y_dir = math.sin(drawState[2])
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_title(title)
    ax.set_xlim([0., ew])
    ax.set_ylim([0., eh])
    ax.grid()
    plt.plot(x_coord[0],y_coord[0], marker = 'o')
    #plt.plot([x_coord[0], x_sightline_f], [y_coord[0], y_sightline_f], color = "red", linestyle = "--", marker = "o")
    #plt.plot([x_coord[0], x_sightline_r], [y_coord[0], y_sightline_r], color = "red", linestyle = "--", marker = "o")
    #q = ax.quiver(x_coord, y_coord, x_sightline_f - float(x_coord[0]), y_sightline_f - float(y_coord[0]), angles = "xy",headwidth=10,headaxislength=5)
    ax.set_aspect('equal', adjustable='box')
    

def main():
  fig1 = plt.figure(1)
  ax = fig1.add_subplot(111)
  ax.set_title("Field")
  ax.set_xlim([0., ew])
  ax.set_ylim([0., eh])
  ax.grid()
  ax.set_aspect('equal', adjustable='box')

  length = 50

  outRecord = []
  env = Enviro(eh, ew)
  ax.plot(env.wheelRobot.state[0],env.wheelRobot.state[1],color = "black",marker = 'o')
  ax.plot([env.wheelRobot.state[0],env.wheelRobot.state[0]+length*math.cos(env.wheelRobot.state[2])],[env.wheelRobot.state[1],env.wheelRobot.state[1]+length*math.sin(env.wheelRobot.state[2])],color = "green")
  for i in range(20):
    out = env.wheelRobot.move(1,0,True)
    outRecord = outRecord + [out]
    ax.plot(env.wheelRobot.state[0],env.wheelRobot.state[1],color = "black",marker = 'o')
    ax.plot([env.wheelRobot.state[0],env.wheelRobot.state[0]+length*math.cos(env.wheelRobot.state[2])],[env.wheelRobot.state[1],env.wheelRobot.state[1]+length*math.sin(env.wheelRobot.state[2])],color = "green")
  for i in range(30):
    out = env.wheelRobot.move(0,1,True)
    outRecord = outRecord + [out]
    ax.plot(env.wheelRobot.state[0],env.wheelRobot.state[1],color = "black",marker = 'o')
    ax.plot([env.wheelRobot.state[0],env.wheelRobot.state[0]+length*math.cos(env.wheelRobot.state[2])],[env.wheelRobot.state[1],env.wheelRobot.state[1]+length*math.sin(env.wheelRobot.state[2])],color = "green")
  for i in range(10):
    out = env.wheelRobot.move(1,1,True)
    outRecord = outRecord + [out]
    ax.plot(env.wheelRobot.state[0],env.wheelRobot.state[1],color = "black",marker = 'o')
    ax.plot([env.wheelRobot.state[0],env.wheelRobot.state[0]+length*math.cos(env.wheelRobot.state[2])],[env.wheelRobot.state[1],env.wheelRobot.state[1]+length*math.sin(env.wheelRobot.state[2])],color = "green")
  for i in range(20):
    out = env.wheelRobot.move(1,2,True)
    outRecord = outRecord + [out]
    ax.plot(env.wheelRobot.state[0],env.wheelRobot.state[1],color = "black",marker = 'o')
    ax.plot([env.wheelRobot.state[0],env.wheelRobot.state[0]+length*math.cos(env.wheelRobot.state[2])],[env.wheelRobot.state[1],env.wheelRobot.state[1]+length*math.sin(env.wheelRobot.state[2])],color = "green")
  for i in range(10):
    out = env.wheelRobot.move(1,1,True)
    outRecord = outRecord + [out]
    ax.plot(env.wheelRobot.state[0],env.wheelRobot.state[1],color = "black",marker = 'o')
    ax.plot([env.wheelRobot.state[0],env.wheelRobot.state[0]+length*math.cos(env.wheelRobot.state[2])],[env.wheelRobot.state[1],env.wheelRobot.state[1]+length*math.sin(env.wheelRobot.state[2])],color = "green")

  OR = np.array(outRecord)
  OR = np.transpose(OR)
  print(OR)

  fig2 = plt.figure(2)
  bx = fig2.add_subplot(111)
  bx.set_title("Df")
  bx.plot(OR[0])
  bx.grid()
  bx.set_xlabel("Time (s)")
  bx.set_ylabel("Distance (mm)")

  fig3 = plt.figure(3)
  bx = fig3.add_subplot(111)
  bx.set_title("Dr")
  bx.plot(OR[1])
  bx.grid()
  bx.set_xlabel("Time (s)")
  bx.set_ylabel("Distance (mm)")
  
  fig4 = plt.figure(4)
  bx = fig4.add_subplot(111)
  bx.set_title("Omega")
  bx.plot(OR[2])
  bx.grid()
  bx.set_xlabel("Time (s)")
  bx.set_ylabel("Angular Velocity (rad/s)")
  
  fig5 = plt.figure(5)
  bx = fig5.add_subplot(111)
  bx.set_title("Bx")
  bx.plot(OR[3])
  bx.grid()
  bx.set_xlabel("Time (s)")
  bx.set_ylabel("Normalized Magnetic Field Strength (G/G)")
  
  fig6 = plt.figure(6)
  bx = fig6.add_subplot(111)
  bx.set_title("By")
  bx.plot(OR[4])
  bx.grid()
  bx.set_xlabel("Time (s)")
  bx.set_ylabel("Normalized Magnetic Field Strength (G/G)")


  #print("X pos: " + str(round(env.wheelRobot.state[0],2)) + "\tY pos: " + str(round(env.wheelRobot.state[1],2)) + "\tTheta: " + str(round(env.wheelRobot.state[2],2)))
  #print("Turning one wheel")
  #env.draw(ew, eh/2, env.wheelRobot.state[0], 0, "Both wheels different speed @ t=0")
  #m1 = env.wheelRobot.move(10,0,True)
  #env.draw(m1[0], m1[1], m1[2], m1[3], "Both wheels different speed @ t=1")
  #m1 = env.wheelRobot.move(10,0,True)
  #env.draw(m1[0], m1[1], m1[2], m1[3], "Both wheels different speed @ t=2")
  #m1 = env.wheelRobot.move(10,0,True)
  #env.draw(m1[0], m1[1], m1[2], m1[3], "Both wheels different speed @ t=3")

  #print("Moving forward")
  #m2 = env.wheelRobot.move(-10,-10,True)
  #env.draw(m2[0], m2[1], m2[2], m2[3],  "Moving forward")

  #print("Pivot")
  #m3 = env.wheelRobot.move(-15,15,True)
  #env.draw(m3[0], m3[1], m3[2], m3[3],  "Pivot")
  


  #for i in range(100):
  #  env.wheelRobot.move(0,1,True)

if __name__ == '__main__':
  main()
  

  