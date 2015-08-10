source ../../devel/setup.bash
rm gui main.o gui.pro mainwindow.o moc_mainwindow.cpp moc_mainwindow.o Makefile
cd ../.. && catkin_make && cd src/se306project
cd gui && qmake -project && qmake -qt4 && make && cd ..
gui/gui
