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

define laser ranger
(
	sensor
	(
		range [ 0.0 30.0 ] # maximum range reported by scanner in meters.
		fov 180 # angular field of view of the scanner, in radians.
		samples 180 # number of laser samples per scan
	)

	# generic model properties
	model
	(
		color "black"
		size [ 0.1 0.1 0.1 ]
	)
)

define human position
(

  # These corresponds to the maximum x y and z of the human in meters.
  size [0.500 0.200 1.700]
  drive "diff"

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

  obstacle_return 1
  ranger_return 1
)

define dog position
(

  # These corresponds to the maximum x y and z of the dog in meters.
  size [1.00 0.300 0.500]
  drive "diff"

  # left hindleg
  block (
  points 4
  point[3] [  0.50   -0.15 ] 
  point[2] [  0.40   -0.15 ] 
  point[1] [  0.40   -0.05 ] 
  point[0] [  0.50   -0.05 ] 
  z [ 0.0 0.200 ]
			)
  # right hindleg
  block (
  points 4
  point[3] [  0.50   0.15 ] 
  point[2] [  0.40   0.15 ] 
  point[1] [  0.40   0.05 ] 
  point[0] [  0.50   0.05 ] 
  z [ 0.0 0.200 ]
			)
  # left foreleg
  block (
  points 4
  point[3] [  -0.40   -0.15 ] 
  point[2] [  -0.30   -0.15 ] 
  point[1] [  -0.30   -0.05 ] 
  point[0] [  -0.40   -0.05 ] 
  z [ 0.0 0.200 ]
			)
  # right foreleg
  block (
  points 4
  point[3] [  -0.40   0.15 ] 
  point[2] [  -0.30   0.15 ] 
  point[1] [  -0.30   0.05 ] 
  point[0] [  -0.40   0.05 ] 
  z [ 0.0 0.200 ]
			)
  # main body
  block (
  points 4
  point[3] [  -0.40  -0.15 ] 
  point[2] [  0.50   -0.15 ] 
  point[1] [  0.50    0.15 ] 
  point[0] [  -0.40   0.15 ] 
  z [ 0.200 0.400 ]
			)
  # head
  block (
  points 4
  point[3] [  -0.50   -0.05 ] 
  point[2] [  -0.40   -0.05 ] 
  point[1] [  -0.40   0.05 ] 
  point[0] [  -0.50   0.05 ] 
  z [ 0.35 0.50 ]
			)

  obstacle_return 1
  ranger_return 1

)

# define newentity oldentity ()
define PickerRobot position
(
  block(
	points 4
	point[0] [0 0]
	point[1] [1 0]
	point[2] [1 1]
	point[3] [0 1]
	z [0 1]
  )
  #properties as name value pairs
  size [2.5 1.5 1]
  drive "diff"
  laser(pose [ 0.050 0.000 0 0.000 ])

  obstacle_return 1
  ranger_return 1

)

define CarrierRobot position
(
  block(
	points 4
	point[0] [0 0]
	point[1] [1 0]
	point[2] [1 1]
	point[3] [0 1]
	z [0 1]
  )
  #properties as name value pairs
  size [1.8 1.5 0.5]
  drive "diff"
  laser(pose [ 0.050 0.000 0 0.000 ])

  obstacle_return 1
  ranger_return 1

)
