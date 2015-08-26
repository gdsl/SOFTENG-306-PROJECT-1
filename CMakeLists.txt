cmake_minimum_required(VERSION 2.8.3)
project(se306project)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	RobotState.msg
	robot_status.msg
	carrier_status.msg
	animal_status.msg
	human_status.msg
	blindperson_status.msg
	weed_status.msg
	gardenworker_status.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES orchard_simulator
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a cpp library
# add_library(orchard_simulator
#   src/${PROJECT_NAME}/orchard_simulator.cpp
# )

## Declare a cpp executable
# add_executable(orchard_simulator_node src/orchard_simulator_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(orchard_simulator_node orchard_simulator_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(orchard_simulator_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS orchard_simulator orchard_simulator_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#catkin_add_gtest(name_of_the_test file the_directory_to_the_test_file cpp_files_that_is_being_tested)
#eg catkin_add_gtest(testRobot test/testRobot.cpp a.cpp b.cpp c.pp)
#catkin_add_gtest(testRobot test/mytest.test test/testRobot.cpp)
#target_link_libraries(testRobot ${catkin_LIBRARIES})

# Libraries
include_directories(include)

#Add executable
add_executable(TallWeed src/TallWeed.cpp)
target_link_libraries(TallWeed Entity ${catkin_LIBRARIES})
add_dependencies(TallWeed se306project_generate_messages_cpp)

add_executable(Tractor src/Tractor.cpp)
target_link_libraries(Tractor Entity ${catkin_LIBRARIES})
add_dependencies(Tractor se306project_generate_messages_cpp)

add_executable(GardenWorker src/GardenWorker.cpp)
target_link_libraries(GardenWorker Person Entity ${catkin_LIBRARIES})
add_dependencies(GardenWorker se306project_generate_message_cpp)

add_executable(BlindPerson src/BlindPerson.cpp)
target_link_libraries(BlindPerson Person Entity ${catkin_LIBRARIES})
add_dependencies(BlindPerson se306project_generate_message_cpp)

add_executable(AlphaDog src/AlphaDog.cpp)
target_link_libraries(AlphaDog Animal Entity ${catkin_LIBRARIES})
add_dependencies(AlphaDog  se306project_generate_messages_cpp)

add_executable(Cat src/Cat.cpp)
target_link_libraries(Cat Animal Entity ${catkin_LIBRARIES})
add_dependencies(Cat  se306project_generate_messages_cpp)

add_executable(EntityMock src/EntityMock.cpp)
target_link_libraries(EntityMock Entity ${catkin_LIBRARIES})
add_dependencies(EntityMock se306project_generate_messages_cpp)

add_executable(Neighbour src/Neighbour.cpp)
target_link_libraries(Neighbour Person Entity ${catkin_LIBRARIES})
add_dependencies(Neighbour se306project_generate_messages_cpp)

add_executable(AlphaPerson src/AlphaPerson.cpp)
target_link_libraries(AlphaPerson Person Entity ${catkin_LIBRARIES})
add_dependencies(AlphaPerson se306project_generate_messages_cpp)

add_executable(PickerRobot src/PickerRobot.cpp)
target_link_libraries(PickerRobot Robot Entity ${catkin_LIBRARIES})
add_dependencies(PickerRobot se306project_generate_messages_cpp)

add_executable(CarrierRobot src/CarrierRobot.cpp)
target_link_libraries(CarrierRobot Robot Entity ${catkin_LIBRARIES})
add_dependencies(CarrierRobot se306project_generate_messages_cpp)

add_executable(Beacon src/Beacon.cpp)
target_link_libraries(Beacon Entity ${catkin_LIBRARIES})
add_dependencies(Beacon se306project_generate_messages_cpp)

#add libraries
add_library(Movement src/Movement.cpp)
add_library(Entity src/Entity.cpp)
add_library(Robot src/Robot.cpp)
add_library(Animal src/Animal.cpp)
add_library(Person src/Person.cpp)
add_library(CatLib src/Cat.cpp)
add_library(EntityMockLib src/EntityMock.cpp)
add_library(AlphaDogLib src/AlphaDog.cpp)
add_library(NeighbourLib src/Neighbour.cpp)
add_library(AlphaPersonLib src/AlphaPerson.cpp)
add_library(PickerRobotLib src/PickerRobot.cpp)
add_library(CarrierRobotLib src/CarrierRobot.cpp)
add_library(BeaconLib src/Beacon.cpp)

#target libraries
target_link_libraries(Entity Movement ${catkin_LIBRARIES})
target_link_libraries(Robot Entity ${catkin_LIBRARIES})
target_link_libraries(Animal Entity ${catkin_LIBRARIES})
target_link_libraries(Person Entity ${catkin_LIBRARIES})
target_link_libraries(AlphaDogLib Animal Entity ${catkin_LIBRARIES})
target_link_libraries(Cat Animal Entity ${catkin_LIBRARIES})
target_link_libraries(EntityMockLib Entity ${catkin_LIBRARIES})
target_link_libraries(Neighbour Person Entity ${catkin_LIBRARIES})
target_link_libraries(AlphaPersonLib Person Entity ${catkin_LIBRARIES})
target_link_libraries(PickerRobotLib Robot Entity ${catkin_LIBRARIES})
target_link_libraries(CarrierRobotLib Robot Entity ${catkin_LIBRARIES})
target_link_libraries(BeaconLib Entity ${catkin_LIBRARIES})



if (CATKIN_ENABLE_TESTING)
	#integration test(everything include testing multiple nodes with stage running)
    find_package(catkin REQUIRED COMPONENTS rostest)
    
    add_rostest_gtest(testAlphaPerson test/rostest/testAlphaPerson.test test/testAlphaPerson.cpp)
    target_link_libraries(testAlphaPerson AlphaPersonLib ${catkin_LIBRARIES})
    
    add_rostest_gtest(testCarrierRobot test/rostest/testCarrierRobot.test test/testCarrierRobot.cpp)
    target_link_libraries(testCarrierRobot CarrierRobotLib ${catkin_LIBRARIES})
    
    add_rostest_gtest(testPickerRobot test/rostest/testPickerRobot.test test/testPickerRobot.cpp)
    target_link_libraries(testPickerRobot PickerRobotLib ${catkin_LIBRARIES})

    add_rostest_gtest(testAlphaDog test/rostest/testAlphaDog.test test/testAlphaDog.cpp)
    target_link_libraries(testAlphaDog AlphaDogLib ${catkin_LIBRARIES})
    
    add_rostest_gtest(testEntityMock test/rostest/testEntityMock.test test/testEntityMock.cpp)
    target_link_libraries(testEntityMock EntityMockLib ${catkin_LIBRARIES})

    add_rostest_gtest(testBeacon test/rostest/testBeacon.test test/testBeacon.cpp)
    target_link_libraries(testBeacon BeaconLib ${catkin_LIBRARIES})

    #add_rostest_gtest(testNeighbour test/rostest/testNeighbour.test test/testNeighbour.cpp)
    #target_link_libraries(testNeighbour NeighbourLib ${catkin_LIBRARIES})
    
    # unit tests
	catkin_add_gtest(testMovement test/testMovement.cpp)
	target_link_libraries(testMovement Movement ${catkin_LIBRARIES})
    
endif()
