/**
 * Generator header file - converts xml documents to world file
 */

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include "GeneratorModel.h"

using namespace std;

<<<<<<< HEAD
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

	vector<float> catPositions;

	queue<float> pickerRobotsPositions;
	queue<float> carrierRobotsPositions;
	queue<float> beaconPositions;
	queue<float> pickerPathPositions;
	queue<float> workerPositions;
	queue<float> gardenerPositions;
	queue<float> dogPositions;
	queue<float> weedPositions;
	queue<float> neighbourPositions;

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
=======
class Generator {
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

        queue<float> catPositions;
        queue<float> pickerRobotsPositions;
        queue<float> carrierRobotsPositions;
        queue<float> beaconPositions; 
        queue<float> pickerPathPositions;
        queue<float> workerPositions;
        queue<float> gardenerPositions;
        queue<float> dogPositions;
        queue<float> weedPositions;
	    queue<float> neighbourPositions;
        queue<float> blindPersonPositions;
        
	private:
		string inputName;
		string outputName;
		ofstream outfile;
		string colourArray[14] = { "PeachPuff", "NavajoWhite", "LemonChiffon", "AliceBlue", "Lavender", "thistle", "LightSalmon", "PaleTurquoise", "PaleGreen", "beige", "plum", "LightGrey", "LightSkyBlue", "SpringGreen" };
		int colourCount = 0;
        int peopleCC = 0;
        int dogCC = 0;
		// Static const variables
		// Distance between trunk/pole and beacon
		int const static SEPARATION = 2;
		// Height and width of bitmap png
		int const static WIDTH = 950;
		int const static HEIGHT = 550;
>>>>>>> 002c2a1f3dc841f56b998f90719b2f96dade691f

};
