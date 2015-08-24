/**
 * Generator header file. Converts xml documents to world file
 */
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include "GeneratorModel.h"

using namespace std;

class Generator
{
	public:
		Generator(const GeneratorModel& model);
		void loadWorld();
		void loadOrchard();
		void loadPickerRobots();
		void loadCarrierRobots();
		void loadPeople();
		void loadAnimals();
        void loadTallWeeds();
        void loadTractor();
        void loadBackdrop();
        void writeLaunchFile();
		void write();
        void calculatePickerPaths();

        GeneratorModel model;

        queue<int> pickerRobotsPositions;
        queue<int> carrierRobotsPositions;
        queue<float> beaconPositions; 
        queue<int> pickerPathPositions;
        queue<float> workerPositions;
        queue<float> gardenerPositions;
        queue<float> dogPositions;
        queue<float> weedPositions;
	vector<int> neighbourPositions;
        
	private:
		string inputName;
		string outputName;
		ofstream outfile;
		string colourArray[14] = { "PeachPuff", "NavajoWhite", "LemonChiffon", "AliceBlue", "Lavender", "thistle", "LightSalmon", "PaleTurquoise", "PaleGreen", "beige", "plum", "LightGrey", "LightSkyBlue", "SpringGreen" };
		int colourCount = 0;
        int peopleCC = 0;
        int dogCC = 0;
		// static const variables
		// distance between trunk/pole and beacon. x coord
		int const static SEPARATION = 2;

		// height and width of bitmap png
		int const static WIDTH = 950;
		int const static HEIGHT = 550;

};
