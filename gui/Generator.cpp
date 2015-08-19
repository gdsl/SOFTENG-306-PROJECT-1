#include "Generator.h"

/**
 * Generator constructor. Takes in input name and output name.
 * Input name specifies XML document to load.
 * Output name specifies world file output name.
 */
Generator::Generator(string inputName, string outputName)
{
	this->inputName = inputName;
	this->outputName = outputName;

	// Load xml document
	doc.LoadFile(inputName.c_str());
	// Return root element
	rootElement = doc.RootElement();
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
	XMLElement* resolutionElement = rootElement -> FirstChildElement("resolution");
	// resolution comment
	outfile << "# set the resolution of the underlying raytrace model in meters" << endl;
	outfile << "resolution " << resolutionElement->GetText() << endl;

	// interval sim
	XMLElement* interval_sim = rootElement -> FirstChildElement("interval_sim");
	outfile << "# simulation timestep in milliseconds" << endl;
	outfile << "interval_sim " << interval_sim->GetText() << endl;

	// interval real
	XMLElement* interval_real = rootElement -> FirstChildElement("interval_real");
	outfile << "# real-time interval between simulation updates in milliseconds" << endl;
	outfile << "interval_real " << interval_real->GetText() << endl;

	XMLElement* paused = rootElement -> FirstChildElement("paused");
	outfile << "paused " << paused->GetText() << endl << endl;

}

/**
 * Loads orchard environment. Places trunk/pole/fruit vine and puts beacon
 */
void Generator::loadOrchard()
{
	XMLElement* models = rootElement -> FirstChildElement("models");

	// orchard element
	XMLElement* orchard = models -> FirstChildElement("orchard");

	// initialise variables
	float rowCount, rowLength, rowWidth, trunkPoleSpacing;

	XMLElement* row_count = orchard -> FirstChildElement("row_count");
	rowCount = atoi(row_count->GetText());

	XMLElement* row_length = orchard -> FirstChildElement("row_length");
	rowLength = atof(row_length->GetText());

	XMLElement* row_width = orchard -> FirstChildElement("row_width");
	rowWidth = atof(row_width->GetText());

	XMLElement* trunk_pole_spacing = orchard -> FirstChildElement("trunk_pole_spacing");
	trunkPoleSpacing = atof(trunk_pole_spacing->GetText());

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

		for (int j = 0; j < rowCount; j++) {

			/*
			 * add beacons at start and end of each row
			 */
			if (i == 0 && j < rowCount-1) {
				outfile << "beacon ( pose [ " << (x - SEPARATION) << " " << (y-rowWidth/2.0) << " 0.000 0.000 ] color \"random\")" << endl;
			} else if (i == columnCount - 1 && j < rowCount-1) {
				outfile << "beacon ( pose [ " << (x + SEPARATION) << " " << (y-rowWidth/2.0) << " 0.000 0.000 ] color \"random\")" << endl;
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
			if (j < rowCount-1) {
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
void Generator::loadRobots()
{
	// Generate robot comment
	outfile << "# Generate robots" << endl;

	XMLElement* models = rootElement->FirstChildElement("models");
	XMLElement* robots = models->FirstChildElement("robots");

	// Picker robot
	XMLElement* picker_number = robots->FirstChildElement("picker_number");
	int pickerNumber = atoi(picker_number->GetText());
    
    string colourArray[9] = { "red", "orange", "yellow", "green", "blue", "purple", "magenta", "aqua", "fuchsia" };
    int colourCount = 0;
	
	// Generate picker robot comment
	outfile << "# Picker robot" << endl;
	int y = 24;
	for (int i = 0; i < pickerNumber; i++) {
		// generate random x and y coord. Robot regions
		// range -25 to 24
		//int x = rand() % 50 - 25;
		int x = -44;
		// range 10 to 20
		//int y = rand() % 10 + 11;
		float theta = 90;
        
        string colour = colourArray[colourCount];
        colourCount += 1;
        
		outfile << "PickerRobot( pose [ " << x << " " << y << " 0 " << theta << " ] name \"Picker" << i+1 << "\" color \"" << colour << "\")" << endl;
		y -= 3;
	}

	// Carrier robot
	XMLElement* carrier_number = robots->FirstChildElement("carrier_number");
	int carrierNumber = atoi(carrier_number->GetText());

	// Generate carrier robot comment
	outfile << "# Carrier robot" << endl;
	for (int i = 0; i < carrierNumber; i++) {
		// generate random x and y coord. Robot regions
		// range -25 to 24
		//int x = rand() % 50 - 25;
		int x = -44;
		// range 10 to 20
		float theta = 90;
        
        string colour = colourArray[colourCount];
        colourCount += 1;
        
		outfile << "CarrierRobot( pose [ " << x << " " << y << " 0 " << theta << " ] name \"Carrier" << i+1 << "\" color \"" << colour << "\")" << endl;
		y -= 3;
	}

	outfile << endl;
}

/**
 * Load people to world file
 */
void Generator::loadPeople()
{
	XMLElement* models = rootElement->FirstChildElement("models");
	XMLElement* people = models->FirstChildElement("people");

	outfile << "# Generate people" << endl;

	// Worker number
	XMLElement* worker_number = people->FirstChildElement("worker_number");
	int workerNumber = atoi(worker_number->GetText());

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
void Generator::loadAnimals()
{
	XMLElement* models = rootElement->FirstChildElement("models");
	XMLElement* animals = models->FirstChildElement("animals");

	outfile << "# Generate animals" << endl;

	// Dog number
	XMLElement* dog_numbers = animals->FirstChildElement("dog_number");
	int dogNumbers = atoi(dog_numbers->GetText());

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
