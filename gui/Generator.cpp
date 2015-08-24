#include "Generator.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include "Markup.h"
#include <QDebug>
#include <math.h> 

/**
 * Generator constructor. Takes in input name and output name.
 * Input name specifies XML document to load.
 * Output name specifies world file output name.
 */
Generator::Generator(const GeneratorModel& model)
{
	this->model = model;

	outfile.open("world/test.world");

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
	// resolution comment
	outfile << "# set the resolution of the underlying raytrace model in meters" << endl;
	outfile << "resolution " << 0.02 << endl;

	// interval sim
	outfile << "# simulation timestep in milliseconds" << endl;
	outfile << "interval_sim " << 100 << endl;

	// interval real
	outfile << "# real-time interval between simulation updates in milliseconds" << endl;
	outfile << "interval_real " << 100 << endl;
    outfile << "paused " << 0 << endl << endl;
}

/**
 * Loads orchard environment. Places trunk/pole/fruit vine and puts beacon
 returns position of beacons
 */
void Generator::loadOrchard()
{
	// Assumption: bitmap is big enough for orchard generation.
	// bitmap image centre is at (0,0,0,0)
	// x and y values used in pose
	double x, y;

	// initial x
	x = -30;
	// initial y
    y = 20.4;    
    
	int columnCount = model.rowLength / model.poleTrunkSpacing;

	// Comments for orchard file
	outfile << "# Orchard and beacon models" << endl;

    for (int i = 0; i < columnCount; i++) {
		    double initialY = y;
		    int beaconCount = 1;

		    // row number
		    outfile << "# row " << i+1 << endl;
        for (int j = 0; j < model.rowCount+1; j++) {
		    /*
		     * add beacons at start and end of each row
		     */
		    if (i == 0 && j < model.rowCount) {
		        beaconPositions.push_back(x - SEPARATION);
		        beaconPositions.push_back(y-model.rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x - SEPARATION) << " " << (y-model.rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << beaconCount << "\" color \"random\")" << endl;
		    } else if (i == columnCount - 1 && j < model.rowCount) {
		        beaconPositions.push_back(x + SEPARATION);
		        beaconPositions.push_back(y-model.rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x + SEPARATION) << " " << (y-model.rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << (beaconCount + 1) << "\" color \"random\")" << endl;
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
		    if (j < model.rowCount) {
			    outfile << "fruitVine( pose [ " << x << " " << (y-model.rowWidth/2.0) << " 1.8 0.000 ] color \"green\")" << endl;
		    }

		    // update y and beaconCount
		    y -= model.rowWidth;
		    beaconCount = beaconCount + 2;
	    }

	    // newline
	    outfile << endl;

	    // reset y
	    y = initialY;

	    // increase x
	    x += model.poleTrunkSpacing;
    }
}


/**
 * Load robots into world file
 */
void Generator::loadPickerRobots()
{
	// Generate robot comment
	outfile << "# Generate robots" << endl;

	// Generate picker robot comment
	outfile << "# Picker robot" << endl;
	int y = 24;
	for (int i = 0; i < model.pickerRobots; i++) {
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
}

void Generator::loadCarrierRobots()
{
    int y = 24;
	// Generate carrier robot comment
	outfile << "# Carrier robot" << endl;
	for (int i = 0; i < model.carrierRobots; i++) {
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
}

/**
 * Load people to world file
 */
void Generator::loadPeople()
{
	outfile << "# Generate people" << endl;
	outfile << "# Generate workers" << endl;
    
    float totalRowWidth = model.rowWidth * 8;
    float yOffset = model.rowWidth / 2;
    
    int columnCount = 70 / model.poleTrunkSpacing;
    int halfColumnCount = columnCount / 2;
    float xOffset = model.poleTrunkSpacing / 2;
    
    int rowEnd = 20 - totalRowWidth;
    int randNumY = rowEnd + 20;
    
    for(int i = 0; i < model.workers; i++) {
        int x = rand() % 76 - 36;
        int y = rand() % randNumY - rowEnd;
        //int x = -30 + (rand() % (40 - -30 +1));
        //int y = rowEnd + (rand() % (20 - rowEnd + 1));
    
       // if( (x > -30) && (x < 40) && (y < 20) && (y > rowEnd)) {
        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset);
        
        int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset);
        workerPositions.push_back(xPos);
        workerPositions.push_back(yPos);
        outfile << "human( pose [ " << xPos << " " << yPos << " 0.000 -90.000 ] name \"Worker" << i+1 << "\" color \"blue\")" << endl;
        /*} else {
            workerPositions.push_back(x);
            workerPositions.push_back(y);
            outfile << "human( pose [ " << x << " " << y << " 0.000 -90.000 ] name \"Worker" << i+1 << "\" color \"blue\")" << endl;
        } */
    }
    
    // Generate gardenworkers
    outfile << "# Generate gardenworkers" << endl;
    for (int i = 0; i < model.gardeners; i++) {
    	// Generate gardenworkers same position as workers
    	//int x = rand() % 82 - 36;
        //int y = rand() % 52 - 26;
        int x = rand() % 76 - 36;
        int y = rand() % randNumY - rowEnd;

        //if( (x > -30) && (x < 40) && (y < 20) && (y > rowEnd)) {
        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset);
        
        int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset);
        gardenerPositions.push_back(xPos);
        gardenerPositions.push_back(yPos);
        outfile << "gardenWorker( pose [ " << xPos << " " << yPos << " 0.000 -90.000 ] name \"GardenWorker" << i+1 << "\" color \"blue\")" << endl;
        /*} else {
            gardenerPositions.push_back(x);
            gardenerPositions.push_back(y);
            outfile << "gardenWorker( pose [ " << x << " " << y << " 0.000 -90.000 ] name \"GardenWorker" << i+1 << "\" color \"blue\")" << endl;
        }*/

    }

	outfile << endl;
}

/**
 * Load animals to world file
 */
void Generator::loadAnimals()
{
	outfile << "# Generate animals" << endl;
	outfile << "# Generate dogs" << endl;
    
    float totalRowWidth = model.rowWidth * 8;
    float yOffset = model.rowWidth / 2;
    
    int columnCount = 70 / model.poleTrunkSpacing;
    int halfColumnCount = columnCount / 2;
    float xOffset = model.poleTrunkSpacing / 2;
    
    int rowEnd = 20 - totalRowWidth;
    int randNumY = rowEnd + 20;
    
	// Generate dogs
    for(int i = 0; i < model.dogs; i++) {
        int x = rand() % 76 - 36;
        int y = rand() % randNumY - rowEnd;
        //int x = rand() % 82 - 36;
        //int y = rand() % 52 - 26;
    
    //    if( (x > -30) && (x < 40) && (y < 20) && (y > rowEnd)) {
        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset);
        
        int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset);
        dogPositions.push_back(xPos);
        dogPositions.push_back(yPos);
        outfile << "dog( pose [ " << xPos << " " << yPos << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"random\")" << endl;
   /*     } else {
            outfile << "dog( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"random\")" << endl;
        }*/
    }
	
	outfile << "# Generate cats" << endl;

	int x = -20;
	// Generate cats
	for(int i = 0; i < model.cats; i++) {
		//outfile << "cat( pose [ -20.000 21.500 0.000 0.000 ] name \"Cat" << i+1 << "\" color \"random\")" << endl;
        	outfile << "cat( pose [ " << x << " 21.500 0.000 0.000 ] name \"Cat" << i+1 << "\" color \"random\")" << endl;
		x += 10;
	}

	outfile << endl;
}

void Generator::loadTallWeeds()
{
//hardcoded to 10 for now - also is 10 in mainwindow.h file
    model.weed = 10;//rand() % 8 + 2; //between 2 and 10
    
    outfile << "#Generate tall weeds" << endl;
    for(int i = 0; i < model.weed; i++){
        int x = rand() % 82 - 36;
        int y = rand() % 52 - 26;
        
        outfile << "tallWeed( pose [ " << x << " " << y << " 0.000 0.000 ] name \"TallWeed" << i+1 << "\" color \"ForestGreen\")" << endl;
    }
}


void Generator::loadTractor() {
    outfile << "#Generate tractor" << endl;
    outfile << "tractor( pose [ 0.00 -10.00 0.000 0.000 ] name \"Tractor\" color \"Blue\")" << endl;
}

void Generator::calculatePickerPaths() {
    //variables to hold start and end beacons for each pickers path
    int nextStartBeacon = 0, nextFinishBeacon = 0;
    float pickersRemaining = float(model.pickerRobots);
    float rowsRemaining = 7.0;
    int numOfRows;
    //for each Picker robot calculate its path to pick kiwifruit
    for (int i = 0; i < model.pickerRobots; i++) {
        //calculate number of rows this Picker should be allocated
        numOfRows = int(ceil(rowsRemaining/pickersRemaining));
        //convert this into Beacon numbers
        //check if the Robot before (if there is one before this one) finishes at an even numbered or odd numbered beacon
        if ((nextFinishBeacon % 2) == 1) {
            nextStartBeacon = nextFinishBeacon + 2;
        } else {
            nextStartBeacon = nextFinishBeacon + 1;
        }
        //if an odd number of rows is to be picked
        if ((numOfRows % 2) == 1) {
            nextFinishBeacon = nextStartBeacon + (numOfRows * 2) - 1;
        }
        //or if an even number of rows is to be picked
        else {
            nextFinishBeacon = nextStartBeacon + (numOfRows * 2) - 2;
        }
        pickerPathPositions.push_back(nextStartBeacon);
        pickerPathPositions.push_back(nextFinishBeacon);

        //update rows and pickers remaining to be allocated
        pickersRemaining = pickersRemaining - 1.0;
        rowsRemaining = rowsRemaining - float(numOfRows);            
    }
}

void Generator::writeLaunchFile(){
    //writes to the launch file
    CMarkup xml;
    bool ok;
    xml.AddElem("launch");
    xml.IntoElem();
    xml.AddElem("node");
    xml.SetAttrib( "name", "stage" );
    xml.SetAttrib( "pkg", "stage_ros" );
    xml.SetAttrib( "type", "stageros" );
    xml.SetAttrib( "args", "$(find se306project)/world/test.world" );
    calculatePickerPaths(); // calculate the picking paths each Picker will take

    int totalObjects = model.getTotalNodes(); // total objects in world

    for (int i = 0; i < totalObjects; i++) {
        xml.AddElem("group");
        ostringstream oss;
        oss << "robot_" << i;
        xml.SetAttrib("ns", oss.str());
        oss.str("");
        oss.clear();
        xml.IntoElem();
        xml.AddElem("node");
        xml.SetAttrib( "pkg", "se306project" );
        if (i < model.weed) { //weeds
            xml.SetAttrib( "name", "TallWeednode" );
            xml.SetAttrib( "type", "TallWeed" );
            int alphaPersonNumber = model.rowCount*2 + model.weed + model.pickerRobots + model.carrierRobots;

            int subscribeIndex = model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers;

            if (model.gardeners > 0) {
            	oss << "/tallweed" << i+1 << "/ " << subscribeIndex << " " << subscribeIndex+model.gardeners-1;
            } else {
            	oss << "/tallweed" << i+1 << "/ -1 -1";
            }

        } else if (i < model.weed + model.beacons) { //beacons
            xml.SetAttrib( "name", "Beaconnode" );
            xml.SetAttrib( "type", "Beacon" );
            int num = i+1-model.weed;
            if (num < 8) {
                num = num * 2 -1;
            } else {
                num = (num - 7) * 2;
            }
            int beaconPos = (i - model.weed)*2;
            qDebug() << beaconPos << " " << beaconPositions.size();
            oss << "/beacon" << num << "/ " << beaconPositions[beaconPos] << " " << beaconPositions[beaconPos+1];
        } else if (i < model.weed + model.beacons + model.pickerRobots) { //picker robots
            xml.SetAttrib( "name", "PickerRobotnode" );
            xml.SetAttrib( "type", "PickerRobot" );
            int pickerPos = (i - model.weed - model.beacons)*2;
            oss << pickerRobotsPositions[pickerPos] << " " << pickerRobotsPositions[pickerPos+1] << " " << pickerPathPositions[pickerPos] << " " << pickerPathPositions[pickerPos+1] << " " << model.rowWidth;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots) { //carriers
            xml.SetAttrib( "name", "CarrierRobotnode" );
            xml.SetAttrib( "type", "CarrierRobot" );
            int carrierPos = (i - model.weed - model.beacons - model.pickerRobots)*2;
            int firstPicker = model.weed + model.beacons;
            int lastPicker = firstPicker + model.pickerRobots - 1;
            int firstCarrier = lastPicker + 1;
            int lastCarrier = firstCarrier + model.carrierRobots - 1;
            oss << carrierRobotsPositions[carrierPos] << " " << carrierRobotsPositions[carrierPos+1] << " " << firstPicker << " " << lastPicker << " " << firstCarrier << " " << lastCarrier;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers) { //AlphaPersons (workers)
            xml.SetAttrib( "name", "AlphaPersonnode" );
            xml.SetAttrib( "type", "AlphaPerson" );
            int workerPos = (i - model.weed - model.beacons - model.pickerRobots - model.carrierRobots)*2;
            oss << workerPositions[workerPos] << " " << workerPositions[workerPos+1];
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners) {
        	xml.SetAttrib("name", "GardenWorkernode");
        	xml.SetAttrib("type", "GardenWorker");
        	if (model.weed > 0) {
        		oss << "/robot_" << i << "/ 0 " << model.weed-1;
        	} else {
        		oss << "/robot_" << i << "/ -1 -1";
        	}
        	
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners + model.dogs) { //dogs
            xml.SetAttrib( "name", "AlphaDognode" );
            xml.SetAttrib( "type", "AlphaDog" );
            int dogPos = (i - model.weed - model.beacons - model.pickerRobots - model.carrierRobots - model.workers - model.gardeners)*2;
            oss << dogPositions[dogPos] << " " << dogPositions[dogPos+1] << " " << model.rowWidth << " " << model.poleTrunkSpacing;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners + model.dogs + model.cats) { //cats
            xml.SetAttrib( "name", "Catnode" );
            xml.SetAttrib( "type", "Cat" );
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners + model.dogs + model.cats + model.tractors) { //tractor
            xml.SetAttrib( "name", "Tractornode" );
            xml.SetAttrib( "type", "Tractor" );
        }
        xml.SetAttrib( "args", oss.str() );
        xml.OutOfElem();
    }   
    xml.OutOfElem();
    xml.Save("launch/test.launch");
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
