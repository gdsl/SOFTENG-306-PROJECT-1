source ../../devel/setup.bash
cd ../.. && catkin_make && cd src/se306project
cd gui && qmake -project && qmake -qt4 && make && cd ..
gui/gui
