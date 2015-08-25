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
  size [0.300 0.300 1.800]
  gui_move 0
  gui_nose 0
)

define pole modelBase
(
  size [0.300 0.300 2.500]
  gui_move 0
  gui_nose 0
)

define verticalFruitVine modelBase
(
    size [0.100 3.500 0.100]
    gui_move 0
    gui_nose 0
)

define backdropTree modelBase
(
  size [0.850 0.850 1.800]
  gui_move 0
  gui_nose 0
)

define backdropTreeTop modelBase
(
  size [1.5 1.5 1.7]
  gui_move 0
  gui_nose 0

  block (
  points 4
  point[3] [ 1 1 ] 
  point[2] [  1 -1 ] 
  point[1] [  -1  -1 ] 
  point[0] [  -1 1 ]  
  z [ 2.5 1.7 ]
  )
  
  block (
  points 4
  point[3] [ 0.5 0.5 ] 
  point[2] [ 0.5 -0.5 ] 
  point[1] [ -0.5 -0.5 ] 
  point[0] [ -0.5 0.5 ]  
  z [ 3 1.7 ]
  )
  
  block (
  points 4
  point[3] [ 0.25 0.25 ] 
  point[2] [ 0.25 -0.25 ] 
  point[1] [ -0.25 -0.25 ] 
  point[0] [ -0.25 0.25 ]  
  z [ 3.25 0.25 ]
  )
            
)

define backdropShrub modelBase
(
  size [0.650 0.650 1.000]
  gui_move 0
  gui_nose 0
)
define horizontalFruitVine modelBase
(
    size [2.500 0.100 0.100]
    gui_move 0
    gui_nose 0
)

define laserBase ranger
(
	sensor
	(
		range [ 0.0 10.0 ] # maximum range reported by scanner in meters.
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

define pretendLaserBase ranger #For animal and humans which should not have lasers
(
	sensor
	(
		size [ 0 0 0 ]
		range [ 0.0 0.0 ] # set to zero
		fov 0
		samples 0
	)
)

define pickerLaser laserBase
()
define carrierLaser laserBase
()
define humanLaser laserBase
()
define animalLaser laserBase
()
define pretendLaser pretendLaserBase
()

define human position
(

  # These corresponds to the maximum x y and z of the human in meters.
  size [0.2 0.500 1.700]
  drive "diff"

  # right leg
  block (
  points 4
  point[3] [ -0.1 -0.05 ] 
  point[2] [  0.1  -0.05] 
  point[1] [  0.1  -0.15] 
  point[0] [  -0.1 -0.15]  
  z [ 0.0 0.700 ]
			)
  # left leg
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1 0.15 ] 
  point[1] [  0.1 0.05 ] 
  point[0] [  -0.1 0.05] 
  z [ 0.0 0.700 ]
			)
  # main body
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1  0.15 ] 
  point[1] [  0.1 -0.15 ] 
  point[0] [  -0.1 -0.15 ] 
  z [ 0.700 1.500 ]
			)
  # left arm
  block (
  points 4
  point[3] [  -0.1   0.25 ] 
  point[2] [  0.1   0.25 ] 
  point[1] [  0.1   0.15 ] 
  point[0] [  -0.1   0.15 ] 
  z [ 0.7 1.500 ]
			)
  # right arm
  block (
  points 4
  point[3] [  -0.1   -0.25 ] 
  point[2] [  0.1   -0.25 ] 
  point[1] [  0.1   -0.15 ] 
  point[0] [  -0.1   -0.15 ] 
  z [ 0.7 1.500 ]
			)
  # head
  #z [lowestDepth highestDepth
  block (
  points 4
  point[3] [  -0.05   0.05 ] 
  point[2] [  0.05   0.05 ] 
  point[1] [  0.05  -0.05 ] 
  point[0] [  -0.05  -0.05 ] 
  z [ 1.5 1.700 ]
			)
  humanLaser(pose [ 0.00 0.000 -1.7 0.000 ])
  obstacle_return 1
  ranger_return 4
)

define neighbour position
(

  # These corresponds to the maximum x y and z of the human in meters.
  size [0.2 0.500 1.700]
  drive "diff"

  # right leg
  block (
  points 4
  point[3] [ -0.1 -0.05 ] 
  point[2] [  0.1  -0.05] 
  point[1] [  0.1  -0.15] 
  point[0] [  -0.1 -0.15]  
  z [ 0.0 0.700 ]
			)
  # left leg
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1 0.15 ] 
  point[1] [  0.1 0.05 ] 
  point[0] [  -0.1 0.05] 
  z [ 0.0 0.700 ]
			)
  # main body
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1  0.15 ] 
  point[1] [  0.1 -0.15 ] 
  point[0] [  -0.1 -0.15 ] 
  z [ 0.700 1.500 ]
			)
  # left arm
  block (
  points 4
  point[3] [  -0.1   0.25 ] 
  point[2] [  0.1   0.25 ] 
  point[1] [  0.1   0.15 ] 
  point[0] [  -0.1   0.15 ] 
  z [ 0.7 1.500 ]
			)
  # right arm
  block (
  points 4
  point[3] [  -0.1   -0.25 ] 
  point[2] [  0.1   -0.25 ] 
  point[1] [  0.1   -0.15 ] 
  point[0] [  -0.1   -0.15 ] 
  z [ 0.7 1.500 ]
			)
  # head
  #z [lowestDepth highestDepth
  block (
  points 4
  point[3] [  -0.05   0.05 ] 
  point[2] [  0.05   0.05 ] 
  point[1] [  0.05  -0.05 ] 
  point[0] [  -0.05  -0.05 ] 
  z [ 1.5 1.700 ]
			)
  humanLaser(pose [ 0.00 0.000 -1.7 0.000 ])
  obstacle_return 1
  ranger_return 4
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
  animalLaser(pose [ 0.500 0.000 -0.500 0.000 ])
  obstacle_return 1
  ranger_return 5

)

define cat position
(

  # These corresponds to the maximum x y and z of the cat in meters.
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
  animalLaser(pose [ 0.500 0.000 -0.500 0.000 ])
  obstacle_return 1
  ranger_return 5

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
	z [0 0.75]
  )
  
  block(
	points 4
	point[0] [0.25 0.25]
	point[1] [0.75 0.25]
	point[2] [0.75 0.75]
	point[3] [0.25 0.75]
	z [1 0.25]
  )
  
  #properties as name value pairs
  size [2.5 1.5 1]
  drive "diff"
  pickerLaser(pose [ 1.25 0.0 -1.0 0.0 ])

  obstacle_return 1
  ranger_return 2

)

define CarrierRobot position
(
  block(
	points 4
	point[0] [0 0]
	point[1] [1 0]
	point[2] [1 1]
	point[3] [0 1]
	z [0 0.75]
  )
  
  block(
	points 4
	point[0] [0.25 0.25]
	point[1] [0.75 0.25]
	point[2] [0.75 0.75]
	point[3] [0.25 0.75]
	z [1 0.25]
  )
  
  #properties as name value pairs
  size [1.8 1.5 0.75]
  drive "diff"
  carrierLaser(pose [ 0.9 0.0 -0.5 0.0 ])

  obstacle_return 1
  ranger_return 3

)

define tractor position
(
  #properties as name value pairs
  size [3 2 1]
  drive "diff"
  pretendLaser(pose [ -500.00 0.000 5 0.000 ])

  obstacle_return 1
  ranger_return 7

    velocity [ 5.0 5.0 0.0 0.0 ] #WHY DOESN'T THIS WORK
 # [ xmin xmax ymin ymax zmin zmax amin amax ]				
    velocity_bounds [-10 10 -10 10 -1 1 -90 90 ]					
    acceleration_bounds [-10 10 -10 10 -1 1 -90 90]
)

define beacon position
(
  size [0.1 0.1 0.1]
  gui_noise 0
  ranger_return 0
  obstacle_return 0
  pretendLaser(pose [ -500.00 0.000 5 0.000 ])
)

define tallWeed position
(
  size [0.3 0.3 1.0]
  obstacle_return 1
  ranger_return 6
  pretendLaser(pose [ 500.00 0.000 5 0.000 ])
)

define gardenWorker position
(

  # These corresponds to the maximum x y and z of the human in meters.
  size [0.2 0.500 1.700]
  drive "diff"

  # right leg
  block (
  points 4
  point[3] [ -0.1 -0.05 ] 
  point[2] [  0.1  -0.05] 
  point[1] [  0.1  -0.15] 
  point[0] [  -0.1 -0.15]  
  z [ 0.0 0.700 ]
			)
  # left leg
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1 0.15 ] 
  point[1] [  0.1 0.05 ] 
  point[0] [  -0.1 0.05] 
  z [ 0.0 0.700 ]
			)
  # main body
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1  0.15 ] 
  point[1] [  0.1 -0.15 ] 
  point[0] [  -0.1 -0.15 ] 
  z [ 0.700 1.500 ]
			)
  # left arm
  block (
  points 4
  point[3] [  -0.1   0.25 ] 
  point[2] [  0.1   0.25 ] 
  point[1] [  0.1   0.15 ] 
  point[0] [  -0.1   0.15 ] 
  z [ 0.7 1.500 ]
			)
  # right arm
  block (
  points 4
  point[3] [  -0.1   -0.25 ] 
  point[2] [  0.1   -0.25 ] 
  point[1] [  0.1   -0.15 ] 
  point[0] [  -0.1   -0.15 ] 
  z [ 0.7 1.500 ]
			)
  # head
  #z [lowestDepth highestDepth
  block (
  points 4
  point[3] [  -0.05   0.05 ] 
  point[2] [  0.05   0.05 ] 
  point[1] [  0.05  -0.05 ] 
  point[0] [  -0.05  -0.05 ] 
  z [ 1.5 1.700 ]
			)
  humanLaser(pose [ 0.00 0.000 -1.7 0.000 ])
  obstacle_return 1
  ranger_return 4
)

define blindPerson position
(
  size [0.2 0.500 1.700]
  drive "diff"

  # right leg
  block (
  points 4
  point[3] [ -0.1 -0.05 ] 
  point[2] [  0.1  -0.05] 
  point[1] [  0.1  -0.15] 
  point[0] [  -0.1 -0.15]  
  z [ 0.0 0.700 ]
			)
  # left leg
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1 0.15 ] 
  point[1] [  0.1 0.05 ] 
  point[0] [  -0.1 0.05] 
  z [ 0.0 0.700 ]
			)
  # main body
  block (
  points 4
  point[3] [  -0.1 0.15 ] 
  point[2] [  0.1  0.15 ] 
  point[1] [  0.1 -0.15 ] 
  point[0] [  -0.1 -0.15 ] 
  z [ 0.700 1.500 ]
			)
  # left arm
  block (
  points 4
  point[3] [  -0.1   0.25 ] 
  point[2] [  0.1   0.25 ] 
  point[1] [  0.1   0.15 ] 
  point[0] [  -0.1   0.15 ] 
  z [ 0.7 1.500 ]
			)
  # right arm
  block (
  points 4
  point[3] [  -0.1   -0.25 ] 
  point[2] [  0.1   -0.25 ] 
  point[1] [  0.1   -0.15 ] 
  point[0] [  -0.1   -0.15 ] 
  z [ 0.7 1.500 ]
			)
  # head
  #z [lowestDepth highestDepth
  block (
  points 4
  point[3] [  -0.05   0.05 ] 
  point[2] [  0.05   0.05 ] 
  point[1] [  0.05  -0.05 ] 
  point[0] [  -0.05  -0.05 ] 
  z [ 1.5 1.700 ]
			)
  humanLaser(pose [ 0.00 0.000 -1.7 0.000 ])
  obstacle_return 1
  ranger_return 1
)
