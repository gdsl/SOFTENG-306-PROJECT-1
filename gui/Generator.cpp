#include "Generator.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>

/**
 * Generator constructor. Takes in input name and output name.
 * Input name specifies XML document to load.
 * Output name specifies world file output name.
 */
Generator::Generator(/*string inputName, */string outputName)
{
	//this->inputName = inputName;
	this->outputName = outputName;

	// Load xml document
	//doc.LoadFile(inputName.c_str());
	// Return root element
	//rootElement = doc.RootElement();
	// Create world file
	outfile.open(outputName.c_str());

	// Comments and loading inc files.
	outfile << "# Authors: Team Test Drive" << endl << endl;

	// include necessary inc files
	outfile << "# Include 'map' and models for world" << endl;
	outfile << "include \"map.inc\"" << endl;
	outfile << "include \"orchard_models.inc\"" << endl << endl;

	// load bitmap
	// Values currently HARD CODED
	outfile << "# load an environment bitmap" << endl;
	outfile << "floorplan" << endl;
	outfile << "(" << endl;
	outfile << "\tname \"Orchard\"" << endl;
	outfile << "\tsize [95 55 0.1]" << endl;
	outfile << "\tpose [0.000 0.000 0.000 0.000]" << endl;
	outfile << "\tbitmap \"bitmaps/orchard.png\"" << endl;
	outfile << ")" << endl << endl;
}

/**
 * Flushes associated buffers and closes file. File is available to be opened by other processes.
 */
void Generator::write()
{
	outfile.close();
}

/**
 * Load environment settings to world file.
 * Note that all these elements are necessary for any world file
 */
void Generator::loadWorld()
{
	// resolution
	//XMLElement* resolutionElement = rootElement -> FirstChildElement("resolution");
	// resolution comment
	outfile << "# set the resolution of the underlying raytrace model in meters" << endl;
	outfile << "resolution " << 0.02 << endl;
	//outfile << "resolution " << resolutionElement->GetText() << endl;

	// interval sim
	//XMLElement* interval_sim = rootElement -> FirstChildElement("interval_sim");
	outfile << "# simulation timestep in milliseconds" << endl;
	outfile << "interval_sim " << 100 << endl;
	//outfile << "interval_sim " << interval_sim->GetText() << endl;

	// interval real
	//XMLElement* interval_real = rootElement -> FirstChildElement("interval_real");
	outfile << "# real-time interval between simulation updates in milliseconds" << endl;
	outfile << "interval_real " << 100 << endl;
	//outfile << "interval_real " << interval_real->GetText() << endl;

	//XMLElement* paused = rootElement -> FirstChildElement("paused");
	//outfile << "paused " << paused->GetText() << endl << endl;
    outfile << "paused " << 0 << endl << endl;
}

/**
 * Loads orchard environment. Places trunk/pole/fruit vine and puts beacon
 */
void Generator::loadOrchard(int rowCount, float rowLength, float rowWidth, float trunkPoleSpacing)
{
	//XMLElement* models = rootElement -> FirstChildElement("models");

	// orchard element
	//XMLElement* orchard = models -> FirstChildElement("orchard");

	// initialise variables
	/*float rowCount, rowLength, rowWidth, trunkPoleSpacing;

	XMLElement* row_count = orchard -> FirstChildElement("row_count");
	rowCount = atoi(row_count->GetText());

	XMLElement* row_length = orchard -> FirstChildElement("row_length");
	rowLength = atof(row_length->GetText());

	XMLElement* row_width = orchard -> FirstChildElement("row_width");
	rowWidth = atof(row_width->GetText());

	XMLElement* trunk_pole_spacing = orchard -> FirstChildElement("trunk_pole_spacing");
	trunkPoleSpacing = atof(trunk_pole_spacing->GetText()); */

	// Assumption: bitmap is big enough for orchard generation.
	// bitmap image centre is at (0,0,0,0)
	// x and y values used in pose
	double x, y;

	// initial x
	//x = -1 * (rowLength/2);
	x = -30;
	// initial y
	//y = (rowCount * rowWidth)/2;
    y = 20.4;    
    
	int columnCount = rowLength / trunkPoleSpacing;

	// Comments for orchard file
	outfile << "# Orchard and beacon models" << endl;

	for (int i = 0; i < columnCount; i++) {
		double initialY = y;

		// row number
		outfile << "# row " << i+1 << endl;

		for (int j = 0; j < rowCount+1; j++) {

			/*
			 * add beacons at start and end of each row
			 */
			if (i == 0 && j < rowCount) {
				outfile << "beacon( pose [ " << (x - SEPARATION) << " " << (y-rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << j << "\" color \"random\")" << endl;
			} else if (i == columnCount - 1 && j < rowCount) {
				outfile << "beacon( pose [ " << (x + SEPARATION) << " " << (y-rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << (j + 7) << "\" color \"random\")" << endl;
			}

			/*
			 * Add trunk at even column, pole on odd column
			 */
			if (i % 2 == 0) {
				outfile << "trunk( pose [ " << x << " " << y << " 0.000 0.000 ] color \"green\")" << endl;
			} else {
				outfile << "pole( pose [ " << x << " " << y << " 0.000 0.000 ] color \"brown\")" << endl;
			}

			/*
			 * add fruit vines except last row
			 * Height is hardcoded to trunk height.
			 */
			if (j < rowCount) {
				outfile << "fruitVine( pose [ " << x << " " << (y-rowWidth/2.0) << " 1.8 0.000 ] color \"green\")" << endl;				
			}

			// update y
			y -= rowWidth;
		}

		// newline
		outfile << endl;

		// reset y
		y = initialY;

		// increase x
		x += trunkPoleSpacing;
	}
}

/**
 * Load robots into world file
 */
std::vector<int> Generator::loadPickerRobots(int pickerNumber)
{
    std::vector<int> pickerRobotsPositions;
	// Generate robot comment
	outfile << "# Generate robots" << endl;

	// Generate picker robot comment
	outfile << "# Picker robot" << endl;
	int y = 24;
	for (int i = 0; i < pickerNumber; i++) {
		// generate random x and y coord. Robot regions
		// range -25 to 24
		//int x = rand() % 50 - 25;
		int x = -42;
		// range 10 to 20
		//int y = rand() % 10 + 11;
		float theta = 90; //0
        
        string colour = colourArray[colourCount];
        colourCount += 1;
        pickerRobotsPositions.push_back(x);
        pickerRobotsPositions.push_back(y);
		outfile << "PickerRobot( pose [ " << x << " " << y << " 0 " << theta << " ] name \"Picker" << i+1 << "\" color \"" << colour << "\")" << endl;
		y -= 4;
	}

    return pickerRobotsPositions;
}

std::vector<int> Generator::loadCarrierRobots(int carrierNumber)
{
    std::vector<int> carrierRobotsPositions;
    int y = 24;
	// Generate carrier robot comment
	outfile << "# Carrier robot" << endl;
	for (int i = 0; i < carrierNumber; i++) {
		int x = -45;
		float theta = 90; //0
        
        string colour = colourArray[colourCount];
        colourCount += 1;
        carrierRobotsPositions.push_back(x);
        carrierRobotsPositions.push_back(y);
		outfile << "CarrierRobot( pose [ " << x << " " << y << " 0 " << theta << " ] name \"Carrier" << i+1 << "\" color \"" << colour << "\")" << endl;
		y -= 3;
	}

	outfile << endl;
	return carrierRobotsPositions;
}

/**
 * Load people to world file
 */
void Generator::loadPeople(int workerNumber, float rowWidth, float spacing)
{
	//XMLElement* models = rootElement->FirstChildElement("models");
	//XMLElement* people = models->FirstChildElement("people");

	outfile << "# Generate people" << endl;

	// Worker number
	//XMLElement* worker_number = people->FirstChildElement("worker_number");
	//int workerNumber = atoi(worker_number->GetText());

	outfile << "# Generate workers" << endl;
    
    float totalRowWidth = rowWidth * 8;
    float yOffset = rowWidth / 2;
    
    int columnCount = 70 / spacing;
    int halfColumnCount = columnCount / 2;
    float xOffset = spacing / 2;
    
    int rowEnd = 20 - totalRowWidth;
    
    for(int i = 0; i < workerNumber; i++) {
        int x = rand() % 82 - 36;
        int y = rand() % 52 - 26;
    
        if( (x > -30) && (x < 40) && (y < 20) && (y > rowEnd)) {
            int xMult = (((rand() % columnCount + 1) * 2) - 1);
            float xPos = -30 + (xMult * xOffset);
        
            int yMult = (((rand() % 8 + 1) * 2) - 1);
            float yPos = 20.4 - (yMult * yOffset);
        
            outfile << "human( pose [ " << xPos << " " << yPos << " 0.000 0.000 ] name \"Worker" << i+1 << "\" color \"blue\")" << endl;
        } else {
            outfile << "human( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Worker" << i+1 << "\" color \"blue\")" << endl;
        }
    }
    
/*	for (int i = 0; i < workerNumber; i++) {
		// generate random x and y coord. People region.
		// range -25 to -1
		int x = rand() % 25 - 25;
		// range -10 to -20
		int y = rand() % 10 - 11;
		outfile << "human( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Worker" << i+1 << "\" color \"blue\")" << endl;
	} */

	outfile << endl;
}

/**
 * Load animals to world file
 */
void Generator::loadAnimals(int dogNumbers, float rowWidth, float spacing)
{
	//XMLElement* models = rootElement->FirstChildElement("models");
	//XMLElement* animals = models->FirstChildElement("animals");

	outfile << "# Generate animals" << endl;

	// Dog number
	//XMLElement* dog_numbers = animals->FirstChildElement("dog_number");
	//int dogNumbers = atoi(dog_numbers->GetText());

	outfile << "# Generate dog" << endl;
    
    float totalRowWidth = rowWidth * 8;
    float yOffset = rowWidth / 2;
    
    int columnCount = 70 / spacing;
    int halfColumnCount = columnCount / 2;
    float xOffset = spacing / 2;
    
    int rowEnd = 20 - totalRowWidth;
    
    for(int i = 0; i < dogNumbers; i++) {
        int x = rand() % 82 - 36;
        int y = rand() % 52 - 26;
    
        if( (x > -30) && (x < 40) && (y < 20) && (y > rowEnd)) {
            int xMult = (((rand() % columnCount + 1) * 2) - 1);
            float xPos = -30 + (xMult * xOffset);
        
            int yMult = (((rand() % 8 + 1) * 2) - 1);
            float yPos = 20.4 - (yMult * yOffset);
        
            outfile << "dog( pose [ " << xPos << " " << yPos << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"random\")" << endl;
        } else {
            outfile << "dog( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"random\")" << endl;
        }
    }
    
    
/*	for (int i = 0; i < dogNumbers; i++) {
		// generate random x and y coord. Animal region.
		// range 0 to 24
		int x = rand() % 25;
		// range -10 to -20
		int y = rand() % 10 - 11;
		outfile << "dog( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"random\")" << endl;
	} */

	outfile << endl;
}

void Generator::loadTallWeeds()
{
    int numTallWeed = rand() % 15 - 5;
    
    outfile << "#Generate tall weeds" << endl;
    for(int i = 0; i < numTallWeed; i++){
        int x = rand() % 82 - 36;
        int y = rand() % 52 - 26;
        
        outfile << "tallWeed( pose [ " << x << " " << y << " 0.000 0.000 ] name \"TallWeed" << i+1 << "\" color \"ForestGreen\")" << endl;
    }
}
/*
int main(int argc, char **argv)
{
	// Create generator file with test values.
	Generator generator("orchard.xml", "test.world");
	generator.loadWorld();
	generator.loadOrchard();
	generator.loadRobots();
	generator.loadPeople();
	generator.loadAnimals();
	generator.write();

	return 0;
}
*/
