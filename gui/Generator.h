/**
 * Generator header file. Converts xml documents to world file
 */
#include "tinyxml2.h"
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <vector>

using namespace tinyxml2;
using namespace std;

class Generator
{
	public:
		Generator(string outputName, int pickerNumber, int carrierNumber, int dogNumber, int workerNumber, float rowWidth, float spacing);
		void loadWorld();
		void loadOrchard();
		void loadPickerRobots();
		void loadCarrierRobots();
		void loadPeople();
		void loadAnimals();
        void loadTallWeeds();
        void writeLaunchFile();
		void write();
		int rowCount = 7;
		float rowLength = 70;
		int pickerNumber;
		int carrierNumber;
		int dogNumber;
		int workerNumber;
		float rowWidth;
		float spacing;
		int numWeeds;
        vector<int> pickerRobotsPositions;
        vector<int> carrierRobotsPositions;
        vector<float> beaconPositions; 
        
	private:
		string inputName;
		string outputName;
		XMLDocument doc;
		XMLElement* rootElement;
		ofstream outfile;
		string colourArray[14] = { "PeachPuff", "NavajoWhite", "LemonChiffon", "AliceBlue", "Lavender", "thistle", "LightSalmon", "PaleTurquoise", "PaleGreen", "beige", "plum", "LightGrey", "LightSkyBlue", "SpringGreen" };
    int colourCount = 0;
		// static const variables
		// distance between trunk/pole and beacon. x coord
		int const static SEPARATION = 2;

		// height and width of bitmap png
		int const static WIDTH = 950;
		int const static HEIGHT = 550;

};
