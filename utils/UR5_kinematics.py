import numpy as np
from numpy import linalg


import cmath
import math
from math import cos as cos
from math import sin as sin
from math import atan2 as atan2
from math import acos as acos
from math import asin as asin
from math import sqrt as sqrt
from math import pi as pi

global mat
mat = np.matrix

global d, a, alph
############################# DH Parameter #############################
#               Link 1   Link 2    Link 3     Link 4      Link 5  Link 6
d    = mat([ 0.089159,       0,        0,   0.10915,    0.09465, 0.0823])
a    = mat([        0,  -0.425, -0.39225,         0,          0,      0])
alph = mat([     pi/2,       0,        0,      pi/2,      -pi/2,      0])


########################## FORWARD KINEMATICS ###########################

#     j : to j coordinate
# theta : joint angle
#   col : which column in joint angle matrix
def Tij(j, theta, col):
  T_a = mat(np.identity(4), copy=False)
  T_a[0, 3] = a[0, j-1]
  T_d = mat(np.identity(4), copy=False)
  T_d[2, 3] = d[0, j-1]

  Rzt = mat([[cos(theta[j-1, col]), -sin(theta[j-1, col]),   0,   0],
	           [sin(theta[j-1, col]),  cos(theta[j-1, col]),   0,   0],
	           [                   0,                     0,   1,   0],
	           [                   0,                     0,   0,   1]],copy=False)    

  Rxa = mat([[1,                 0,                  0,   0],
			       [0, cos(alph[0, j-1]), -sin(alph[0, j-1]),   0],
			       [0, sin(alph[0, j-1]),  cos(alph[0, j-1]),   0],
			       [0,                 0,                  0,   1]],copy=False)

  A_i = T_d * Rzt * T_a * Rxa
	    
  return A_i

# theta : joint angle
#   col : which column in joint angle matrix
def FK(theta, col):  
  A_1 = Tij(1, theta, col)
  A_2 = Tij(2, theta, col)
  A_3 = Tij(3, theta, col)
  A_4 = Tij(4, theta, col)
  A_5 = Tij(5, theta, col)
  A_6 = Tij(6, theta, col)
      
  T_06 = A_1 * A_2 * A_3 * A_4 * A_5 * A_6

  return T_06[0:3, 3]

########################## INVERSE KINEMATICS ########################### 

def IK(desired_pos):
  theta = mat(np.zeros((6, 8)))
  P_05 = (desired_pos * mat([0, 0, -d[0, 5], 1]).T - mat([0, 0, 0, 1]).T)
  
  #### Theta 1 ####
  # Singular : d4 > sqrt(P_05[1, 0]^2 + P_05[0, 0]^2)
  ## Add checkJ1 function ##
  psi = atan2(P_05[1, 0], P_05[0, 0])
  phi = acos(d[0, 3] / sqrt(P_05[1, 0]**2 + P_05[0, 0]**2))

  # left shoulder or right shoulder
  theta[0, 0:4] = pi/2 + psi + phi
  theta[0, 4:8] = pi/2 + psi - phi

  theta = theta.real
  
  #### Theta 5 ####
  # wrist up or wrist down
  cl = [0, 4]
  for i in range(0, len(cl)):
    col = cl[i]
    T_10 = linalg.inv(Tij(1, theta, col))
    T_16 = T_10 * desired_pos
    theta[4,   col:col+2] = + acos((T_16[2, 3] - d[0, 3]) / d[0, 5])
    theta[4, col+2:col+4] = - acos((T_16[2, 3] - d[0, 3]) / d[0, 5])

  theta = theta.real
  
  #### Theta 6 ####
  cl = [0, 2, 4, 6]
  for i in range(0, len(cl)):
    col = cl[i]
    T_10 = linalg.inv(Tij(1, theta, col))
    T_16 = linalg.inv(T_10 * desired_pos)
    # Singular : sin(theta5) = 0 or T_16(1,3) = T_16(2,3) = 0
    ## Add checkJ5 function ##
    theta[5, col:col+2] = atan2((-T_16[1, 2] / sin(theta[4, col])), (T_16[0, 2] / sin(theta[4, col])))

  theta = theta.real

  #### Theta 3 ####
  cl = [0, 2, 4, 6]
  for i in range(0, len(cl)):
    col = cl[i]
    T_10 = linalg.inv(Tij(1, theta, col))
    T_65 = Tij(6, theta, col)
    T_54 = Tij(5, theta, col)
    T_14 = (T_10 * desired_pos) * linalg.inv(T_54 * T_65)
    P_13 = T_14 * mat([0, -d[0, 3], 0, 1]).T - mat([0, 0, 0, 1]).T
    # Singular : linalg.norm(P_13)**2 - a[0, 1]**2 - a[0, 2]**2) / (2 * a[0, 1] * a[0, 2]) \in [-1, 1]
    ## Add checkJ3 function ##
    t3 = cmath.acos((linalg.norm(P_13)**2 - a[0, 1]**2 - a[0, 2]**2) / (2 * a[0, 1] * a[0, 2]))
    theta[2,   col] =  t3.real
    theta[2, col+1] = -t3.real

  #### Theta 2 & Theta 4 ####
  cl = [0, 1, 2, 3, 4, 5, 6, 7]
  for i in range(0,len(cl)):
    col = cl[i]
    T_10 = linalg.inv(Tij(1, theta, col))
    T_65 = linalg.inv(Tij(6, theta, col))
    T_54 = linalg.inv(Tij(5, theta, col))
    T_14 = (T_10 * desired_pos) * T_65 * T_54
    P_13 = T_14 * mat([0, -d[0, 3], 0, 1]).T - mat([0, 0, 0, 1]).T
    
    #### Theta 2 ####
    theta[1, col] = -atan2(P_13[1], -P_13[0]) + asin(a[0, 2] * sin(theta[2, col]) / linalg.norm(P_13))
    #### Theta 4 ####
    T_32 = linalg.inv(Tij(3, theta, col))
    T_21 = linalg.inv(Tij(2, theta, col))
    T_34 = T_32 * T_21 * T_14
    theta[3, col] = atan2(T_34[1, 0], T_34[0, 0])
    theta = theta.real
  
  #################### For CoppeliaSim ####################
  iniPos = mat([pi/2, pi/2, 0, pi/2, 0, 0]).T
  for r in range(6):
    for c in range(8):
      theta[r, c] = theta[r, c] + iniPos[r]
      if theta[r, c] > pi:
        theta[r, c] = theta[r, c] - 2*pi
      elif theta[r, c] < -pi:
        theta[r, c] = theta[r, c] + 2*pi
  return theta

def euler_to_rot_matrix(euler_Angle):
    roll  = euler_Angle[0]
    pitch = euler_Angle[1]
    yaw   = euler_Angle[2]

    R_x = np.array([[1,            0,             0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])

    R_y = np.array([[ np.cos(pitch),  0, np.sin(pitch)],
                    [             0,  1,             0],
                    [-np.sin(pitch),  0, np.cos(pitch)]])

    R_z = np.array([[np.cos(yaw), -np.sin(yaw),  0],
                    [np.sin(yaw),  np.cos(yaw),  0],
                    [          0,            0,  1]])

    rot_matrix = np.dot(R_z, np.dot(R_y, R_x))

    return rot_matrix

def rot_trans_to_matrix(rot_mat, trans_vec):
    transformation_matrix = np.zeros((4, 4))
    transformation_matrix[:3, :3] = rot_mat
    transformation_matrix[:3, 3] = trans_vec
    transformation_matrix[3, 3] = 1
    return transformation_matrix


if __name__ == '__main__':
  point = np.array([ 0.6, -0.1, 0.245])
  Orinn = np.array([0, pi, pi])
  R_matrix = euler_to_rot_matrix(Orinn)
  print(rot_trans_to_matrix(R_matrix, point))