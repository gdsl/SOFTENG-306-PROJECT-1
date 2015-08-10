source ../../devel/setup.bash
cd ../.. && catkin_make && cd src/se306project
cd gui && rm gui main.o gui.pro mainwindow.o moc_mainwindow.cpp moc_mainwindow.o Makefile && cd ..
cd gui && qmake -project && qmake -qt4 && make && cd ..
gui/gui
