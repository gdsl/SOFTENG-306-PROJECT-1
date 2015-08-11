#include "worker.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

void Worker::newLabel() {
	exec("gui/getStatus.sh");
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
				if (s.compare(0, 5, "pos_x") == 0) {
	 				emit requestNewLabel(buffer, 0); //emits a signal
				//	cout << buffer;
				//	ui->robotPanel1->setPlainText(buffer);
				} else if (s.compare(0, 5, "pos_y") == 0) {
	 				emit requestNewLabel(buffer, 1); //emits a signal
				//	cout << buffer;
				}
			}
		}
		pclose(stream);
	}
}
