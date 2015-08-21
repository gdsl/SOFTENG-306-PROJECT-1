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
 returns position of beacons
 */
std::vector<float> Generator::loadOrchard(int rowCount, float rowLength, float rowWidth, float trunkPoleSpacing)
{
    std::vector<float> beaconPositions;

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
		    int beaconCount = 1;

		    // row number
		    outfile << "# row " << i+1 << endl;
        for (int j = 0; j < rowCount+1; j++) {
		    /*
		     * add beacons at start and end of each row
		     */
		    if (i == 0 && j < rowCount) {
		        beaconPositions.push_back(x - SEPARATION);
		        beaconPositions.push_back(y-rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x - SEPARATION) << " " << (y-rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << beaconCount << "\" color \"random\")" << endl;
		    } else if (i == columnCount - 1 && j < rowCount) {
		        beaconPositions.push_back(x + SEPARATION);
		        beaconPositions.push_back(y-rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x + SEPARATION) << " " << (y-rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << (beaconCount + 1) << "\" color \"random\")" << endl;
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

		    // update y and beaconCount
		    y -= rowWidth;
		    beaconCount = beaconCount + 2;
	    }

	    // newline
	    outfile << endl;

	    // reset y
	    y = initialY;

	    // increase x
	    x += trunkPoleSpacing;
    }
	
	return beaconPositions;
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
void Generator::loadPeople(int workerNumber)
{
	//XMLElement* models = rootElement->FirstChildElement("models");
	//XMLElement* people = models->FirstChildElement("people");

	outfile << "# Generate people" << endl;

	// Worker number
	//XMLElement* worker_number = people->FirstChildElement("worker_number");
	//int workerNumber = atoi(worker_number->GetText());

	outfile << "# Generate workers" << endl;
	for (int i = 0; i < workerNumber; i++) {
		// generate random x and y coord. People region.
		// range -25 to -1
		int x = rand() % 25 - 25;
		// range -10 to -20
		int y = rand() % 10 - 11;
		outfile << "human( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Worker" << i+1 << "\" color \"blue\")" << endl;
	}

	outfile << endl;
}

/**
 * Load animals to world file
 */
void Generator::loadAnimals(int dogNumbers)
{
	//XMLElement* models = rootElement->FirstChildElement("models");
	//XMLElement* animals = models->FirstChildElement("animals");

	outfile << "# Generate animals" << endl;

	// Dog number
	//XMLElement* dog_numbers = animals->FirstChildElement("dog_number");
	//int dogNumbers = atoi(dog_numbers->GetText());

	outfile << "# Generate dog" << endl;
	for (int i = 0; i < dogNumbers; i++) {
		// generate random x and y coord. Animal region.
		// range 0 to 24
		int x = rand() % 25;
		// range -10 to -20
		int y = rand() % 10 - 11;
		outfile << "dog( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"random\")" << endl;
	}

	outfile << endl;
}

void Generator::loadTallWeeds()
{
    outfile << "#Generate tall weeds" << endl;
    for(int i = 0; i < 10; i++){
        int x = rand() % 92 - 42;
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
