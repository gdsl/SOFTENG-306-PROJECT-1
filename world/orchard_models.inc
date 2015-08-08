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

  # These corresponds to the maximum x y and z of the human in meters.
  size [0.500 0.200 1.700]

  # right leg
  block (
  points 4
  point[3] [  0.15   -0.1 ] 
  point[2] [  0.05   -0.1 ] 
  point[1] [  0.05   0.1 ] 
  point[0] [  0.15   0.1 ] 
  z [ 0.0 0.700 ]
			)
  # left leg
  block (
  points 4
  point[3] [  -0.15   -0.1 ] 
  point[2] [  -0.05   -0.1 ] 
  point[1] [  -0.05   0.1 ] 
  point[0] [  -0.15   0.1 ] 
  z [ 0.0 0.700 ]
			)
  # main body
  block (
  points 4
  point[3] [  -0.15   -0.1 ] 
  point[2] [  0.15   -0.1 ] 
  point[1] [  0.15   0.1 ] 
  point[0] [  -0.15   0.1 ] 
  z [ 0.700 1.500 ]
			)
  # left arm
  block (
  points 4
  point[3] [  -0.25   -0.1 ] 
  point[2] [  -0.15   -0.1 ] 
  point[1] [  -0.15   0.1 ] 
  point[0] [  -0.25   0.1 ] 
  z [ 0.7 1.500 ]
			)
  # right arm
  block (
  points 4
  point[3] [  0.25   -0.1 ] 
  point[2] [  0.15   -0.1 ] 
  point[1] [  0.15   0.1 ] 
  point[0] [  0.25   0.1 ] 
  z [ 0.7 1.500 ]
			)
  # head
  block (
  points 4
  point[3] [  0.05   0.05 ] 
  point[2] [  0.05   -0.05 ] 
  point[1] [  -0.05  0.05 ] 
  point[0] [  -0.05  -0.05 ] 
  z [ 1.5 1.700 ]
			)

)

define dog modelBase
(

  # These corresponds to the maximum x y and z of the human in meters.
  size [0.500 0.200 1.700]

  # right leg
  block (
  points 4
  point[3] [  0.15   -0.1 ] 
  point[2] [  0.05   -0.1 ] 
  point[1] [  0.05   0.1 ] 
  point[0] [  0.15   0.1 ] 
  z [ 0.0 0.700 ]
			)
  # left leg
  block (
  points 4
  point[3] [  -0.15   -0.1 ] 
  point[2] [  -0.05   -0.1 ] 
  point[1] [  -0.05   0.1 ] 
  point[0] [  -0.15   0.1 ] 
  z [ 0.0 0.700 ]
			)
  # main body
  block (
  points 4
  point[3] [  -0.15   -0.1 ] 
  point[2] [  0.15   -0.1 ] 
  point[1] [  0.15   0.1 ] 
  point[0] [  -0.15   0.1 ] 
  z [ 0.700 1.500 ]
			)
  # left arm
  block (
  points 4
  point[3] [  -0.25   -0.1 ] 
  point[2] [  -0.15   -0.1 ] 
  point[1] [  -0.15   0.1 ] 
  point[0] [  -0.25   0.1 ] 
  z [ 0.7 1.500 ]
			)
  # right arm
  block (
  points 4
  point[3] [  0.25   -0.1 ] 
  point[2] [  0.15   -0.1 ] 
  point[1] [  0.15   0.1 ] 
  point[0] [  0.25   0.1 ] 
  z [ 0.7 1.500 ]
			)
  # head
  block (
  points 4
  point[3] [  0.05   0.05 ] 
  point[2] [  0.05   -0.05 ] 
  point[1] [  -0.05  0.05 ] 
  point[0] [  -0.05  -0.05 ] 
  z [ 1.5 1.700 ]
			)

)
