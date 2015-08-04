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
