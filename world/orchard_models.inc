# 
# Desc : This file defines all the models needed for the orchard.world
# Authors : Team Test Drive
#

#all objects should be detected by lasers
define modelBase model
(
    obstacle_return 1
    laser_return 1
    ranger_return 1
)

define trunk modelBase
(
  size [0.500 0.500 1.800]
  gui_nose 0
)

define pole modelBase
(
  size [0.400 0.400 2.500]
  gui_nose 0
)

define human modelBase
(

  # These corresponds to the maximum x y and z of the table in meters.
  size [0.300 0.300 1.800]

  # right leg
  block (
  points 4
  point[3] [  0.15   -0.05 ] 
  point[2] [  0.05   -0.05 ] 
  point[1] [  0.05   0.05 ] 
  point[0] [  0.15   0.05 ] 
  z [ 0.0 0.900 ]
			)
  # left leg
  block (
  points 4
  point[3] [  -0.15   -0.05 ] 
  point[2] [  -0.05   -0.05 ] 
  point[1] [  -0.05   0.05 ] 
  point[0] [  -0.15   0.05 ] 
  z [ 0.0 0.900 ]
			)

  

)
