#include "worker.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <QDebug>
#include <fstream>
#include <unistd.h>
#include <signal.h>
#include <sstream>

using namespace std;

void Worker::setId(string id_string) {
    //Set fields defined in the worker.h header files.
    stringId = id_string;
	id = QString::fromStdString(id_string);
}

void Worker::executeScript() {
   // qDebug("executeScript prerun");
   // string filePath = "gui/rostopicScripts/robot_" + stringId + ".sh";
   // ofstream myfile;
   // myfile.open (filePath.c_str());
   // myfile << "cd ../../../..\nsource devel/setup.bash\nrostopic echo /robot_" << stringId.c_str() << "/status";
   // myfile.close();
   // string cmd = "chmod +x " + filePath;
   // system(cmd.c_str());
	exec("rostopic echo /robot_" + stringId + "/status");
}

//for processing command
void Worker::exec(string cmd) {
	string data;
	FILE * stream;
	const int max_buffer = 256;
	char buffer[max_buffer];

	stream = popen(cmd.c_str(), "r");
	if (stream) {
		while (!feof(stream)) {
			if (fgets(buffer, max_buffer, stream) != NULL) {
				string s = string(buffer);
				buffer[strlen(buffer) - 1] = '\0';
				if (s.compare(0, 5, "pos_x") == 0) { //check if line starts with pos_x
	 				emit requestNewLabel(id, buffer, 1); //emits a signal
				} else if (s.compare(0, 5, "pos_y") == 0) {//check if line starts with pos_y
	 				emit requestNewLabel(id, buffer, 2); 
				} else if (s.compare(0, 9, "pos_theta") == 0) {//check if line starts with pos_theta
					emit requestNewLabel(id, buffer, 3); 
				} else if (s.compare(0, 6, "status") == 0) {//check if line starts with status
					emit requestNewLabel(id, buffer, 4); 
				} else if (s.compare(0, 8, "obstacle") == 0) {
					emit requestNewLabel(id, buffer, 5); 
				}
			} 
		}
		pclose(stream);
	}
}

void Worker::setMainWindow(MainWindow *m) {
    mw = m;
}

//for sending to Tractor
void Worker::sendToTractor() {
    FILE *in;
    ostringstream oss;
    oss << "../../devel/lib/se306project/Tractor " << mw->getTotalNodesFromModel();
    if (!(in = popen(oss.str().c_str(), "w"))) {
        qDebug("failed to run tractor node");
        return;
    }
    while (1){
        int last = mw->getLastKeyPressed();
        if (last == 0) {
            fputs("none",in);
        } else if (last == 1) {
            fputs("left",in);
        } else if (last == 2) {
            fputs("right",in);
        } else if (last == 3) {
            fputs("up",in);
        } else if (last == 4) {
            fputs("down",in);
        } 
        fputs("\n",in);
        
        int j =fflush(in);
        //qDebug() << j;
        //qDebug("wrote");
        usleep(1000000);
    }
    pclose(in);
}


