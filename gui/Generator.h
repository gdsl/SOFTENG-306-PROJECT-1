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
		Generator(/*string inputName, */string outputName);
		void loadWorld();
		void loadOrchard(int rowCount, float rowLength, float rowWidth, float trunkPoleSpacing);
		std::vector<int> loadPickerRobots(int pickerNumber);
		std::vector<int> loadCarrierRobots(int carrierNumber);
		void loadPeople(int workerNumber);
		void loadAnimals(int dogNumbers, float rowWidth, float spacing);
        void loadTallWeeds();
		void write();

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
