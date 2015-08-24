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
Generator::Generator(const GeneratorModel& model) {
	this->model = model;
	outfile.open("world/test.world");

	// Comments and loading inc files.
	outfile << "# Authors: Team Test Drive" << endl << endl;

	// Include necessary inc files
	outfile << "# Include 'map' and models for world" << endl;
	outfile << "include \"map.inc\"" << endl;
	outfile << "include \"orchard_models.inc\"" << endl << endl;

	// Load bitmap
	// Balues currently HARD CODED
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
void Generator::write() {
	outfile.close();
}

/**
 * Load environment settings to world file.
 * Note that all these elements are necessary for any world file
 */
void Generator::loadWorld() {
	// Resolution comment
	outfile << "# set the resolution of the underlying raytrace model in meters" << endl;
	outfile << "resolution " << 0.02 << endl;

	// Interval sim
	outfile << "# simulation timestep in milliseconds" << endl;
	outfile << "interval_sim " << 100 << endl;

	// Interval real
	outfile << "# real-time interval between simulation updates in milliseconds" << endl;
	outfile << "interval_real " << 100 << endl;
    outfile << "paused " << 0 << endl << endl;
}

/**
 * Loads orchard environment. Places trunk/pole/fruit vine and puts beacon
 returns position of beacons
 */
void Generator::loadOrchard() {
	// Assumption: bitmap is big enough for orchard generation
	// Bitmap image centre is at (0,0,0,0)
	// x and y values used in pose
	double x, y;

	// Initial x
	x = -30;
	// Initial y
   	y = 20.4;    
    
	int columnCount = model.rowLength / model.poleTrunkSpacing;

	// Comments for orchard file
	outfile << "# Orchard and beacon models" << endl;

    for (int i = 0; i < columnCount; i++) {
		    double initialY = y;
		    int beaconCount = 1;

		    // Row number
		    outfile << "# row " << i+1 << endl;
        for (int j = 0; j < model.rowCount+1; j++) {
		    //add beacons at start and end of each row
		    if (i == 0 && j < model.rowCount) {
		        beaconPositions.push(x - SEPARATION);
		        beaconPositions.push(y-model.rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x - SEPARATION) << " " << (y-model.rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << beaconCount << "\" color \"random\")" << endl;
		    } else if (i == columnCount - 1 && j < model.rowCount) {
		        beaconPositions.push(x + SEPARATION);
		        beaconPositions.push(y-model.rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x + SEPARATION) << " " << (y-model.rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << (beaconCount + 1) << "\" color \"random\")" << endl;
		    }
		    // Add trunk at even column, pole on odd column
		    if (i % 2 == 0) {
			    outfile << "trunk( pose [ " << x << " " << y << " 0.000 0.000 ] color \"SaddleBrown\")" << endl;
		    } else {
			    outfile << "pole( pose [ " << x << " " << y << " 0.000 0.000 ] color \"black\")" << endl;
		    }
		    // Add vertical fruit vines except last row, and place 3 vertical vines in each column.
		    // Height is hardcoded to trunk height.
		    if (j < model.rowCount) {
			    outfile << "verticalFruitVine( pose [ " << x << " " << (y-model.rowWidth/2.0) << " 1.8 0.000 ] size [0.100 " << model.rowWidth << " 0.100] color \"ForestGreen\")" << endl;
                // Place 2 more vertical vines on the right of each tree, just not on the very last tree
                if (i < (columnCount - 1)) {
                    outfile << "verticalFruitVine( pose [ " << (x + model.poleTrunkSpacing/3) << " " << (y-model.rowWidth/2.0) << " 1.8 0.000 ] size [0.100 " << model.rowWidth << " 0.100] color \"ForestGreen\")" << endl;
                    outfile << "verticalFruitVine( pose [ " << (x + (model.poleTrunkSpacing/3)*2) << " " << (y-model.rowWidth/2.0) << " 1.8 0.000 ] size [0.100 " << model.rowWidth << " 0.100] color \"ForestGreen\")" << endl;
                }               
		    } 
		// Add 3 horizontal fruit vines except first column.
		// Centre placement of each vine is halfway between current column and previous column.
            if (i != 0) {
                outfile << "horizontalFruitVine( pose [ " << (x-model.poleTrunkSpacing/2) << " " << y 
                    << " 1.8 0.000 ] size ["<< model.poleTrunkSpacing << " 0.100 0.100] color \"ForestGreen\")" << endl;
                if (j < model.rowCount) {
                    outfile << "horizontalFruitVine( pose [ " << (x-model.poleTrunkSpacing/2) << " " << (y-model.rowWidth/3) 
                        << " 1.8 0.000 ] size ["<< model.poleTrunkSpacing << " 0.100 0.100] color \"ForestGreen\")" << endl;
                    outfile << "horizontalFruitVine( pose [ " << (x-model.poleTrunkSpacing/2) << " " << (y-(model.rowWidth/3)*2) 
                        << " 1.8 0.000 ] size ["<< model.poleTrunkSpacing << " 0.100 0.100] color \"ForestGreen\")" << endl;
                }
            }
		    // Update y and beaconCount
		    y -= model.rowWidth;
		    beaconCount = beaconCount + 2;
	    }

	    // Newline
	    outfile << endl;
	    // Reset y
	    y = initialY;
	    // Increase x
	    x += model.poleTrunkSpacing;
    }
}

void Generator::loadBackdrop() {
    outfile << "#Generate Backdrop Trees" << endl;
    string backdropColourArray[7] = {"lawn green", "LimeGreen", "ForestGreen", "YellowGreen", "OliveDrab", "DarkOliveGreen", "DarkGreen"};
    double x, y;
    // Initial x
	x = -47;
	// Initial y
    y = 27;
    
    int colourRand = 0;
    string colour = "";
    
    for (int i = 0; i < 27; i++) {
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropTree( pose [ " << x << " " << y << " 0.000 0.000 ] color \"SaddleBrown\")" << endl;
        outfile << "backdropTreeTop( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        y -= 1.1;
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropShrub( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        y -= 0.9;
    }
    
    for (int i = 0; i < 47; i++) {
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropTree( pose [ " << x << " " << y << " 0.000 0.000 ] color \"SaddleBrown\")" << endl;
        outfile << "backdropTreeTop( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        x += 1.1;
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropShrub( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        x += 0.9;
    }
    
    for (int i = 0; i < 27; i++) {
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropTree( pose [ " << x << " " << y << " 0.000 0.000 ] color \"SaddleBrown\")" << endl;
        outfile << "backdropTreeTop( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        y += 1.1;
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropShrub( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        y += 0.9;
    }
    
    for (int i = 0; i < 47; i++) {
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropTree( pose [ " << x << " " << y << " 0.000 0.000 ] color \"SaddleBrown\")" << endl;
        outfile << "backdropTreeTop( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        x -= 1.1;
        colourRand = rand() % 6;
        colour = backdropColourArray[colourRand];
        outfile << "backdropShrub( pose [ " << x << " " << y << " 0.000 0.000 ] color \"" + colour + "\")" << endl;
        x -= 0.9;
    }
}


/**
 * Load robots into world file
 */
void Generator::loadPickerRobots() {
	// Generate robot comment
	outfile << "# Generate robots" << endl;

	// Generate picker robot comment
	outfile << "# Picker robot" << endl;
	int y = 24;
	for (int i = 0; i < model.pickerRobots; i++) {
		// Generate random x and y coordinates
		int x = -42;
		float theta = 90;
        
        string colour = colourArray[colourCount];
        colourCount += 1;
        pickerRobotsPositions.push(x);
        pickerRobotsPositions.push(y);
		outfile << "PickerRobot( pose [ " << x << " " << y << " 0 " << theta << " ] name \"Picker" << i+1 << "\" color \"" << colour << "\")" << endl;
		y -= 4;
	}
}

void Generator::loadCarrierRobots() {
    int y = 24;
	// Generate carrier robot comment
	outfile << "# Carrier robot" << endl;
	for (int i = 0; i < model.carrierRobots; i++) {
		int x = -45;
		float theta = 90;
        
        string colour = colourArray[colourCount];
        colourCount += 1;
        carrierRobotsPositions.push(x);
        carrierRobotsPositions.push(y);
		outfile << "CarrierRobot( pose [ " << x << " " << y << " 0 " << theta << " ] name \"Carrier" << i+1 << "\" color \"" << colour << "\")" << endl;
		y -= 3;
	}

	outfile << endl;
}

/**
 * Load people to world file
 */
void Generator::loadPeople() {
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

        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset);
        
        int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset);

        workerPositions.push(xPos);
        workerPositions.push(yPos);
        
        string colour = colourArray[peopleCC];
        peopleCC += 1;
        
        outfile << "human( pose [ " << xPos << " " << yPos << " 0.000 -90.000 ] name \"Worker" << i+1 << "\" color \"" + colour + "\")" << endl;
    }
    
    // Generate garden workers
    outfile << "# Generate gardenworkers" << endl;
    for (int i = 0; i < model.gardeners; i++) {
    	// Generate gardenworkers same position as workers
        int x = rand() % 76 - 36;
        int y = rand() % randNumY - rowEnd;

        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset);
        
        int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset);

        gardenerPositions.push(xPos);
        gardenerPositions.push(yPos);
        
        string colour = colourArray[peopleCC];
        peopleCC += 1;
        
        outfile << "gardenWorker( pose [ " << xPos << " " << yPos << " 0.000 -90.000 ] name \"GardenWorker" << i+1 << "\" color \"" + colour + "\")" << endl;
        
    }
        // Generate neighbours
        outfile << "# Generate neighbours" << endl;
for (int i = 0; i < model.neighbours; i++) {
    	// Generate the position of the neigbours
        int x = rand() % 76 - 36;
        int y = rand() % randNumY - rowEnd;

        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset);
        int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset);
        string colour = colourArray[peopleCC];
        peopleCC += 1;
        neighbourPositions.push(xPos);
        neighbourPositions.push(yPos);
        outfile << "neighbour( pose [ " << xPos << " " << yPos << " 0.000 -90.000 ] name \"neighbour" << i+1 << "\" color \"" + colour + "\")" << endl;
    }
    
    // Generate blind people
    outfile << "# Generate blind people" << endl;
    
    for(int i = 0; i < model.blindPerson; i++) {
        int x = 0;
        int y = 0;
        string colour = colourArray[peopleCC];
        peopleCC += 1;
        blindPersonPositions.push(x);
        blindPersonPositions.push(y);
        outfile << "blindPerson( pose [ " << x << " " << y << " 0.000 0.000 ] name \"BlindPerson" << i+1 << "\" color \"" + colour + "\")" << endl;
    }
	outfile << endl;
}


/**
 * Load animals to world file
 */
void Generator::loadAnimals() {
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
        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset);
        int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset);
        
        dogPositions.push(xPos);
        dogPositions.push(yPos);

        string colour = colourArray[dogCC];
        dogCC += 1;
        
        outfile << "dog( pose [ " << xPos << " " << yPos << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"" + colour + "\")" << endl;
    }
	
	outfile << "# Generate cats" << endl;

	int x = -30;
	// Generate cats
	for(int i = 0; i < model.cats; i++) {
		x += model.poleTrunkSpacing * 4;
		catPositions.push(x);
        	outfile << "cat( pose [ " << x << " 21.500 0.000 0.000 ] name \"Cat" << i+1 << "\" color \"random\")" << endl;
	}

	outfile << endl;
}

void Generator::loadTallWeeds() {
// Hardcoded to 10 for now - also is 10 in mainwindow.h file
    model.weed = 10;
    
    outfile << "#Generate tall weeds" << endl;
    for(int i = 0; i < model.weed; i++){
        int x = rand() % 82 - 36;
        int y = rand() % 52 - 26;
        weedPositions.push(x);
        weedPositions.push(y);
        outfile << "tallWeed( pose [ " << x << " " << y << " 0.000 0.000 ] name \"TallWeed" << i+1 << "\" color \"ForestGreen\")" << endl;
    }
}

void Generator::loadTractor() {
    outfile << "#Generate tractor" << endl;
    outfile << "tractor( pose [ 0.00 -10.00 0.000 0.000 ] name \"Tractor\" color \"Blue\")" << endl;
}

void Generator::calculatePickerPaths() {
    // Variables to hold start and end beacons for each pickers path
    int nextStartBeacon = 0, nextFinishBeacon = 0;
    float pickersRemaining = float(model.pickerRobots);
    float rowsRemaining = 7.0;
    int numOfRows;
    // For each Picker robot, calculate its path to pick kiwifruit
    for (int i = 0; i < model.pickerRobots; i++) {
        // Calculate number of rows this Picker should be allocated
        numOfRows = int(ceil(rowsRemaining/pickersRemaining));
        // Convert this into Beacon numbers
        // Check if the Robot before (if there is one before this one) finishes at an even numbered or odd numbered beacon
        if ((nextFinishBeacon % 2) == 1) {
            nextStartBeacon = nextFinishBeacon + 2;
        } else {
            nextStartBeacon = nextFinishBeacon + 1;
        }
        // If an odd number of rows is to be picked
        if ((numOfRows % 2) == 1) {
            nextFinishBeacon = nextStartBeacon + (numOfRows * 2) - 1;
        }
        // Or if an even number of rows is to be picked
        else {
            nextFinishBeacon = nextStartBeacon + (numOfRows * 2) - 2;
        }
        pickerPathPositions.push(nextStartBeacon);
        pickerPathPositions.push(nextFinishBeacon);

        // Update rows and pickers remaining to be allocated
        pickersRemaining = pickersRemaining - 1.0;
        rowsRemaining = rowsRemaining - float(numOfRows);            
    }
}

void Generator::writeLaunchFile() {
    // Writes to the launch file
    CMarkup xml;
    bool ok;
    xml.AddElem("launch");
    xml.IntoElem();
    xml.AddElem("node");
    xml.SetAttrib( "name", "stage" );
    xml.SetAttrib( "pkg", "stage_ros" );
    xml.SetAttrib( "type", "stageros" );
    xml.SetAttrib( "args", "$(find se306project)/world/test.world" );
    // Calculate the picking paths each Picker will take
    calculatePickerPaths(); 

    // Total objects in the world
    int totalObjects = model.getTotalNodes(); 

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
        int firstPicker = model.weed + model.beacons;
        int lastPicker = firstPicker + model.pickerRobots - 1;
        int firstCarrier = lastPicker + 1;
        int lastCarrier = firstCarrier + model.carrierRobots - 1;

	// Weeds
        if (i < model.weed) {
            xml.SetAttrib( "name", "TallWeednode" );
            xml.SetAttrib( "type", "TallWeed" );
            int alphaPersonNumber = model.rowCount*2 + model.weed + model.pickerRobots + model.carrierRobots;
            int subscribeIndex = model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers;
            float xPos = weedPositions.front();
            weedPositions.pop();
            float yPos = weedPositions.front();
            weedPositions.pop();
            if (model.gardeners > 0) {
            	oss << xPos << " " << yPos << " /tallweed" << i+1 << "/ " << subscribeIndex << " " << subscribeIndex+model.gardeners-1;
            } else {
            	oss << xPos << " " << yPos << " /tallweed" << i+1 << "/ -1 -1";
            }
	// Beacons
        } else if (i < model.weed + model.beacons) { 
            xml.SetAttrib( "name", "Beaconnode" );
            xml.SetAttrib( "type", "Beacon" );
            int num = i+1-model.weed;
            if (num < 8) {
                num = num * 2 -1;
            } else {
                num = (num - 7) * 2;
            }
            float xPos = beaconPositions.front();
            beaconPositions.pop();
            float yPos = beaconPositions.front();
            beaconPositions.pop();
            oss << xPos << " " << yPos << " /beacon" << num << "/ ";
	// Picker robots
        } else if (i < model.weed + model.beacons + model.pickerRobots) { 
            xml.SetAttrib( "name", "PickerRobotnode" );
            xml.SetAttrib( "type", "PickerRobot" );
            float xPos = pickerRobotsPositions.front();
            pickerRobotsPositions.pop();
            float yPos = pickerRobotsPositions.front();
            pickerRobotsPositions.pop();
            float nextStart = pickerPathPositions.front();
            pickerPathPositions.pop();
            float nextFinish = pickerPathPositions.front();
            pickerPathPositions.pop();            
            float theta = 0;
            oss << xPos << " " << yPos << " " << theta << " " << nextStart << " " << nextFinish << " " << model.rowWidth << " " << firstCarrier << " " << lastCarrier;
	// Carrier robots
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots) { 
            xml.SetAttrib( "name", "CarrierRobotnode" );
            xml.SetAttrib( "type", "CarrierRobot" );
            float xPos = carrierRobotsPositions.front();
            carrierRobotsPositions.pop();
            float yPos = carrierRobotsPositions.front();
            carrierRobotsPositions.pop();
            oss << xPos << " " << yPos << " " << firstPicker << " " << lastPicker << " " << firstCarrier << " " << lastCarrier;
	// Workers
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers) { 
            xml.SetAttrib( "name", "AlphaPersonnode" );
            xml.SetAttrib( "type", "AlphaPerson" );
            float xPos = workerPositions.front();
            workerPositions.pop();
            float yPos = workerPositions.front();
            workerPositions.pop();
            oss << xPos << " " << yPos << " " << firstPicker << " " << lastCarrier;
	// Gardeners
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners) {
        	xml.SetAttrib("name", "GardenWorkernode");
        	xml.SetAttrib("type", "GardenWorker");
        	if (model.weed > 0) {
        		oss << "/robot_" << i << "/ 0 " << model.weed-1;
        	} else {
        		oss << "/robot_" << i << "/ -1 -1";
        	}
	// Neighbours
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers+ model.gardeners+model.neighbours) { 
            xml.SetAttrib( "name", "Neighbournode" );
            xml.SetAttrib( "type", "Neighbour" );
            float x = neighbourPositions.front();
            neighbourPositions.pop();
            float y = neighbourPositions.front();
            neighbourPositions.pop();
            float theta = 0;
            oss << x << " " << y << " " << theta;
	// Blind people
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners +model.neighbours + model.blindPerson) {
            xml.SetAttrib( "name", "BlindPersonnode");
            xml.SetAttrib( "type", "BlindPerson");
            float xPos = blindPersonPositions.front();
            blindPersonPositions.pop();
            float yPos = blindPersonPositions.front();
            blindPersonPositions.pop();
            float theta = 0;
            oss << xPos << " " << yPos << " " << theta;
	// Dogs
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.blindPerson + model.gardeners +model.neighbours + model.dogs) { 
            xml.SetAttrib( "name", "AlphaDognode" );
            xml.SetAttrib( "type", "AlphaDog" );
            float xPos = dogPositions.front();
            dogPositions.pop();
            float yPos = dogPositions.front();
            dogPositions.pop();
            float theta = 0;
    		oss << xPos << " " << yPos << " " << theta << " " << model.rowWidth << " " << model.poleTrunkSpacing;
	// Cats
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners + model.blindPerson + model.neighbours + model.dogs + model.cats) {
            xml.SetAttrib( "name", "Catnode" );
            xml.SetAttrib( "type", "Cat" );
            float xPos = catPositions.front();
            catPositions.pop();
            float yPos = 21.5;
            float theta = 0;
		    oss << xPos << " " << yPos << " " << theta << " " << model.poleTrunkSpacing;
	// Tractor
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners + model.blindPerson +model.neighbours+ model.dogs + model.cats + model.tractors) {
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
