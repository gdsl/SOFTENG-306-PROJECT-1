#include "worker.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

void Worker::setId(string id_string) {
    //Set fields defined in the worker.h header files.
    stringId = id_string;
	id = QString::fromStdString(id_string);
}

void Worker::executeScript() {
    qDebug("executeScript prerun");
	exec("gui/rostopicScripts/robot_" + stringId + ".sh");
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
				}
			} 
		}
		pclose(stream);
	}
}
