#include "worker.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

void GUIUpdater::newLabel(const QString &image) {
	exec("gui/getStatus.sh");

}

//for processing command
void GUIUpdater::exec(string cmd) {
	string data;
	FILE * stream;
	const int max_buffer = 256;
	char buffer[max_buffer];

	stream = popen(cmd.c_str(), "r");
	if (stream) {
		while (!feof(stream)) {
			if (fgets(buffer, max_buffer, stream) != NULL) {
				string s = string(buffer);
				if (s.compare(0, 5, "pos_x: 4.0") == 0) {
	 				emit requestNewLabel(buffer); //emits a signal
					cout << buffer;
				//	ui->robotPanel1->setPlainText(buffer);
				} else if (s.compare(0, 5, "pos_y") == 0) {
					cout << buffer;
				}
			}
		}
		pclose(stream);
	}
}
