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
		        beaconPositions.push(x - SEPARATION);
		        beaconPositions.push(y-model.rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x - SEPARATION) << " " << (y-model.rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << beaconCount << "\" color \"random\")" << endl;
		    } else if (i == columnCount - 1 && j < model.rowCount) {
		        beaconPositions.push(x + SEPARATION);
		        beaconPositions.push(y-model.rowWidth/2.0);
			    outfile << "beacon( pose [ " << (x + SEPARATION) << " " << (y-model.rowWidth/2.0) << " 0.000 0.000 ] name \"beacon" << (beaconCount + 1) << "\" color \"random\")" << endl;
		    }

		    /*
		     * Add trunk at even column, pole on odd column
		     */
		    if (i % 2 == 0) {
			    outfile << "trunk( pose [ " << x << " " << y << " 0.000 0.000 ] color \"SaddleBrown\")" << endl;
		    } else {
			    outfile << "pole( pose [ " << x << " " << y << " 0.000 0.000 ] color \"black\")" << endl;
		    }

		    /*
		     * add vertical fruit vines except last row
             * place 3 vertical vines in each column
		     * Height is hardcoded to trunk height.
             */
		    if (j < model.rowCount) {
			    outfile << "verticalFruitVine( pose [ " << x << " " << (y-model.rowWidth/2.0) << " 1.8 0.000 ] size [0.050 " << model.rowWidth << " 0.100] color \"PaleGreen\")" << endl;
                
                // place 2 more vertical vines on the right of each tree, just not on the very last tree.
                if (i < (columnCount - 1)) {
                    outfile << "verticalFruitVine( pose [ " << (x + model.poleTrunkSpacing/3) << " " << (y-model.rowWidth/2.0) << " 1.8 0.000 ] size [0.050 " << model.rowWidth << " 0.100] color \"PaleGreen\")" << endl;
                    outfile << "verticalFruitVine( pose [ " << (x + (model.poleTrunkSpacing/3)*2) << " " << (y-model.rowWidth/2.0) << " 1.8 0.000 ] size [0.050 " << model.rowWidth << " 0.100] color \"PaleGreen\")" << endl;
                }               
                
		    } 
            
            /*
             * add 3 horizontal fruit vines except first column
             * centre placement of each vine is halfway between current column and previous column
		     */
            if (i != 0) {
                outfile << "horizontalFruitVine( pose [ " << (x-model.poleTrunkSpacing/2) << " " << y 
                    << " 1.8 0.000 ] size ["<< model.poleTrunkSpacing << " 0.050 0.100] color \"PaleGreen\")" << endl;
                if (j < model.rowCount) {
                    outfile << "horizontalFruitVine( pose [ " << (x-model.poleTrunkSpacing/2) << " " << (y-model.rowWidth/3) 
                        << " 1.8 0.000 ] size ["<< model.poleTrunkSpacing << " 0.050 0.100] color \"PaleGreen\")" << endl;
                    outfile << "horizontalFruitVine( pose [ " << (x-model.poleTrunkSpacing/2) << " " << (y-(model.rowWidth/3)*2) 
                        << " 1.8 0.000 ] size ["<< model.poleTrunkSpacing << " 0.050 0.100] color \"PaleGreen\")" << endl;
                }
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

void Generator::loadBackdrop()
{
    outfile << "#Generate Backdrop Trees" << endl;
    
    string backdropColourArray[7] = {"lawn green", "LimeGreen", "ForestGreen", "YellowGreen", "OliveDrab", "DarkOliveGreen", "DarkGreen"};
    
    double x, y;
    // initial x
	x = -47;
	// initial y
    y = 27;
    
    int colourRand = 0;
    string colour = "";
    
    for(int i = 0; i < 27; i++) {
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
    
    for(int i = 0; i < 47; i++) {
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
    
    for(int i = 0; i < 27; i++) {
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
    
    for(int i = 0; i < 47; i++) {
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
        pickerRobotsPositions.push(x);
        pickerRobotsPositions.push(y);
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

        workerPositions.push(xPos);
        workerPositions.push(yPos);
        
        string colour = colourArray[peopleCC];
        peopleCC += 1;
        
        outfile << "human( pose [ " << xPos << " " << yPos << " 0.000 -90.000 ] name \"Worker" << i+1 << "\" color \"" + colour + "\")" << endl;
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
    	//int x = rand() % 82 - 36;
        //int y = rand() % 52 - 26;
        //int x = 24;
        //int y = -55;

        //if( (x > -30) && (x < 40) && (y < 20) && (y > rowEnd)) {
        //int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = 43 ;
        //int yMult = (((rand() % 8 + 1) * 2) - 1);
        float yPos = 19;
        yPos=yPos-(i * model.rowWidth);
        string colour = colourArray[peopleCC];
        peopleCC += 1;
        neighbourPositions.push(xPos);
        neighbourPositions.push(yPos);
        outfile << "neighbour( pose [ " << xPos << " " << yPos << " 0.000 -90.000 ] name \"neighbour" << i+1 << "\" color \"" + colour + "\")" << endl;
    }
    
    //blind persons
    outfile << "# Generate blind people" << endl;
    
    for(int i = 0; i < model.blindPerson; i++) {
        int x = rand() % 76 - 36;
        int y = rand() % randNumY - rowEnd;
        int xMult = (((rand() % columnCount + 1) * 2) - 1);
        float xPos = -30 + (xMult * xOffset) + 0.5;
        int yMult = (((rand() % 6) * 2) - 1);
        float yPos = 20.4 - (yMult * yOffset) + 0.5;
        
        blindPersonPositions.push_back(xPos);
        blindPersonPositions.push_back(yPos);

        string colour = colourArray[peopleCC];
        peopleCC += 1;
        
        outfile << "blindPerson( pose [ " << xPos << " " << yPos << " 0.000 0.000 ] name \"BlindPerson" << i+1 << "\" color \"" + colour + "\")" << endl;
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
        float xPos, yPos;
        
        if (i*2 < blindPersonPositions.size()) {
            xPos = blindPersonPositions[2*i] - 0.5;
            yPos = blindPersonPositions[2*i+1] - 0.5;            
        } else {
            int x = rand() % 76 - 36;
            int y = rand() % randNumY - rowEnd;
            int xMult = (((rand() % columnCount + 1) * 2) - 1);
            xPos = -30 + (xMult * xOffset);
            int yMult = (((rand() % 6) * 2) - 1);
            yPos = 20.4 - (yMult * yOffset);
        }

        dogPositions.push(xPos);
        dogPositions.push(yPos);

        string colour = colourArray[animalCC];
        animalCC += 1;
        
        outfile << "dog( pose [ " << xPos << " " << yPos << " 0.000 0.000 ] name \"Dog" << i+1 << "\" color \"" + colour + "\")" << endl;
    }
	
	outfile << "# Generate cats" << endl;

	int x = -30;
	// Generate cats
	for(int i = 0; i < model.cats; i++) {
		x += model.poleTrunkSpacing * 4;
		catPositions.push(x);
        string colour = colourArray[animalCC];
        animalCC += 1;
		//outfile << "cat( pose [ -20.000 21.500 0.000 0.000 ] name \"Cat" << i+1 << "\" color \"random\")" << endl;
        	outfile << "cat( pose [ " << x << " 21.500 0.000 0.000 ] name \"Cat" << i+1 << "\" color \"" + colour + "\")" << endl;
	}

	outfile << endl;
}

void Generator::loadTallWeeds()
{
//hardcoded to 10 for now - also is 10 in mainwindow.h file
    model.weed = 10;//rand() % 8 + 2; //between 2 and 10
    
    outfile << "#Generate tall weeds" << endl;
    for(int i = 0; i < model.weed; i++){
        int x = rand() % 76 - 30;
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
        pickerPathPositions.push(nextStartBeacon);
        pickerPathPositions.push(nextFinishBeacon);

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
    int firstPicker = model.weed + model.beacons;
    int lastPicker = firstPicker + model.pickerRobots - 1;
    int firstCarrier = lastPicker + 1;
    int lastCarrier = firstCarrier + model.carrierRobots - 1;
    int firstDog = lastCarrier + 1 +  model.workers + model.blindPerson + model.gardeners + model.neighbours;
    int lastDog = firstDog + model.dogs - 1;
    int dogCounter = firstDog;

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
            float xPos = weedPositions.front();
            weedPositions.pop();
            float yPos = weedPositions.front();
            weedPositions.pop();
            
            if (model.gardeners > 0) {
            	oss << xPos << " " << yPos << " /tallweed" << i+1 << "/ " << subscribeIndex << " " << subscribeIndex+model.gardeners-1;
            } else {
            	oss << xPos << " " << yPos << " /tallweed" << i+1 << "/ -1 -1";
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
            float xPos = beaconPositions.front();
            beaconPositions.pop();
            float yPos = beaconPositions.front();
            beaconPositions.pop();
            oss << xPos << " " << yPos << " /beacon" << num << "/ ";
        } else if (i < model.weed + model.beacons + model.pickerRobots) { //picker robots
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
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots) { //carriers
            xml.SetAttrib( "name", "CarrierRobotnode" );
            xml.SetAttrib( "type", "CarrierRobot" );
            float xPos = carrierRobotsPositions.front();
            carrierRobotsPositions.pop();
            float yPos = carrierRobotsPositions.front();
            carrierRobotsPositions.pop();
            oss << xPos << " " << yPos << " " << firstPicker << " " << lastPicker << " " << firstCarrier << " " << lastCarrier;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers) { //AlphaPersons (workers)
            xml.SetAttrib( "name", "AlphaPersonnode" );
            xml.SetAttrib( "type", "AlphaPerson" );
            float xPos = workerPositions.front();
            workerPositions.pop();
            float yPos = workerPositions.front();
            workerPositions.pop();
            oss << xPos << " " << yPos << " " << firstPicker << " " << lastCarrier;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners) {
        	xml.SetAttrib("name", "GardenWorkernode");
        	xml.SetAttrib("type", "GardenWorker");

        	int robotStartPos = model.weed + model.beacons;
        	int robotEndPos = model.weed + model.beacons + model.pickerRobots + model.carrierRobots - 1;

        	int gardenWorkerStartPos = model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers;
        	int gardenWorkerEndPos = model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners - 1;

        	if (model.pickerRobots + model.carrierRobots > 0) {
        		oss << robotStartPos << " " << robotEndPos << " " << gardenWorkerStartPos << " " << gardenWorkerEndPos << " " << i;
        	} else {
        		oss << "-1 -1" << gardenWorkerStartPos << " " << gardenWorkerEndPos << " " << i;
        	}
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers+ model.gardeners+model.neighbours) { //neighbours
            xml.SetAttrib( "name", "Neighbournode" );
            xml.SetAttrib( "type", "Neighbour" );
            float x = neighbourPositions.front();
            neighbourPositions.pop();
            float y = neighbourPositions.front();
            neighbourPositions.pop();
            float theta = 0;
            oss << x << " " << y << " " << theta;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners +model.neighbours + model.blindPerson) {
            xml.SetAttrib( "name", "BlindPersonnode");
            xml.SetAttrib( "type", "BlindPerson");
            float xPos = blindPersonPositions.front();
            blindPersonPositions.pop_front();
            float yPos = blindPersonPositions.front();
            blindPersonPositions.pop_front();
            float theta = 0;                          
            int dogNum = dogCounter;
            if (dogCounter > lastDog) {
                dogNum = -1;
            }
            oss << xPos << " " << yPos << " " << theta << " " << dogNum;
            dogCounter++;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.blindPerson + model.gardeners +model.neighbours + model.dogs) { //dogs
            xml.SetAttrib( "name", "AlphaDognode" );
            xml.SetAttrib( "type", "AlphaDog" );
            float xPos = dogPositions.front();
            dogPositions.pop();
            float yPos = dogPositions.front();
            dogPositions.pop();
            float theta = 0;
    		oss << xPos << " " << yPos << " " << theta << " " << model.rowWidth << " " << model.poleTrunkSpacing;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners + model.blindPerson + model.neighbours + model.dogs + model.cats) { //cats
            xml.SetAttrib( "name", "Catnode" );
            xml.SetAttrib( "type", "Cat" );
            float xPos = catPositions.front();
            catPositions.pop();
            float yPos = 21.5;
            float theta = 0;
		    oss << xPos << " " << yPos << " " << theta << " " << model.poleTrunkSpacing;
        } else if (i < model.weed + model.beacons + model.pickerRobots + model.carrierRobots + model.workers + model.gardeners + model.blindPerson +model.neighbours+ model.dogs + model.cats + model.tractors) { //tractor
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

