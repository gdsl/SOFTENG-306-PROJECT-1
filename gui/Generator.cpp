#include "Generator.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>
#include "Markup.h"
#include <math.h> 
/**
 * Generator constructor. Takes in input name and output name.
 * Input name specifies XML document to load.
 * Output name specifies world file output name.
 */
Generator::Generator(string outputName, int pickerNumber, int carrierNumber, int dogNumber, int catNumber, int workerNumber, float rowWidth, float spacing)
{
	this->outputName = outputName;
	this->pickerNumber = pickerNumber;
	this->carrierNumber = carrierNumber;
	this->dogNumber = dogNumber;
	this->catNumber = catNumber;
	this->workerNumber = workerNumber;
	this->rowWidth = rowWidth;
	this->spacing = spacing;

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
    
	int columnCount = rowLength / spacing;

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
	    x += spacing;
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
}

void Generator::loadCarrierRobots()
{
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
}

/**
 * Load people to world file
 */
void Generator::loadPeople()
{
	outfile << "# Generate people" << endl;
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
    
	outfile << endl;
}

/**
 * Load animals to world file
 */
void Generator::loadAnimals()
{
	outfile << "# Generate animals" << endl;
	outfile << "# Generate dogs" << endl;
    
    float totalRowWidth = rowWidth * 8;
    float yOffset = rowWidth / 2;
    
    int columnCount = 70 / spacing;
    int halfColumnCount = columnCount / 2;
    float xOffset = spacing / 2;
    
    int rowEnd = 20 - totalRowWidth;
    
	// Generate dogs
    for(int i = 0; i < dogNumber; i++) {
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
	
	outfile << "# Generate cats" << endl;

	// Generate cats
	for(int i = 0; i < catNumber; i++) {
        int x = rand() % 82 - 36;
        int y = rand() % 52 - 26;
    
        if( (x > -30) && (x < 40) && (y < 20) && (y > rowEnd)) {
            int xMult = (((rand() % columnCount + 1) * 2) - 1);
            float xPos = -30 + (xMult * xOffset);
        
            int yMult = (((rand() % 8 + 1) * 2) - 1);
            float yPos = 20.4 - (yMult * yOffset);
        
            outfile << "cat( pose [ " << xPos << " " << yPos << " 0.000 0.000 ] name \"Cat" << i+1 << "\" color \"random\")" << endl;
        } else {
            outfile << "cat( pose [ " << x << " " << y << " 0.000 0.000 ] name \"Cat" << i+1 << "\" color \"random\")" << endl;
        }
    }

	outfile << endl;
}

void Generator::loadTallWeeds()
{
//hardcoded to 10 for now - also is 10 in mainwindow.h file
    numWeeds = 10;//rand() % 8 + 2; //between 2 and 10
    
    outfile << "#Generate tall weeds" << endl;
    for(int i = 0; i < numWeeds; i++){
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
    float pickersRemaining = float(pickerNumber);
    float rowsRemaining = 7.0;
    int numOfRows;
    //for each Picker robot calculate its path to pick kiwifruit
    for (int i = 0; i < pickerNumber; i++) {
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
    int numBeacons = rowCount * 2;
    int totalObjects = numWeeds + numBeacons + pickerNumber + carrierNumber + dogNumber + catNumber + workerNumber + 1; // 1 tractor

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
        
        if (i < numWeeds) { //weeds
            xml.SetAttrib( "name", "TallWeednode" );
            xml.SetAttrib( "type", "TallWeed" );
            int alphaPersonNumber = rowCount*2 + numWeeds + pickerNumber + carrierNumber;
            oss << "/tallweed" << i+1 << "/ /robot_" << alphaPersonNumber << "/status";
        } else if (i < numWeeds + numBeacons) { //beacons
            xml.SetAttrib( "name", "Beaconnode" );
            xml.SetAttrib( "type", "Beacon" );
            int num = i+1-numWeeds;
            if (num < 8) {
                num = num * 2 -1;
            } else {
                num = (num - 7) * 2;
            }
            int beaconPos = (i - numWeeds)*2;
            oss << "/beacon" << num << "/ " << beaconPositions[beaconPos] << " " << beaconPositions[beaconPos+1];
        } else if (i < numWeeds + numBeacons + pickerNumber) { //picker robots
            xml.SetAttrib( "name", "PickerRobotnode" );
            xml.SetAttrib( "type", "PickerRobot" );
            int pickerPos = (i - numWeeds - numBeacons)*2;
            oss << pickerRobotsPositions[pickerPos] << " " << pickerRobotsPositions[pickerPos+1] << " " << pickerPathPositions[pickerPos] << " " << pickerPathPositions[pickerPos+1];
        } else if (i < numWeeds + numBeacons + pickerNumber + carrierNumber) { //carriers
            xml.SetAttrib( "name", "CarrierRobotnode" );
            xml.SetAttrib( "type", "CarrierRobot" );
            int carrierPos = (i - numWeeds - numBeacons - pickerNumber)*2;
            int firstPicker = numWeeds + numBeacons;
            int lastPicker = firstPicker + pickerNumber - 1;
            oss << carrierRobotsPositions[carrierPos] << " " << carrierRobotsPositions[carrierPos+1] << " " << firstPicker << " " << lastPicker;
        } else if (i < numWeeds + numBeacons + pickerNumber + carrierNumber + workerNumber) { //AlphaPersons (workers)
            xml.SetAttrib( "name", "AlphaPersonnode" );
            xml.SetAttrib( "type", "AlphaPerson" );
        } else if (i < numWeeds + numBeacons + pickerNumber + carrierNumber + workerNumber + dogNumber) { //dogs
            xml.SetAttrib( "name", "AlphaDognode" );
            xml.SetAttrib( "type", "AlphaDog" );
<<<<<<< HEAD
        } else if (i < numWeeds + numBeacons + pickerNumber + carrierNumber + workerNumber + dogNumber + catNumber) { //cats
            xml.SetAttrib( "name", "Catnode" );
            xml.SetAttrib( "type", "Cat" );
	}
=======
        } else if (i < numWeeds + numBeacons + pickerNumber + carrierNumber + workerNumber + dogNumber + 1) { //tractor
            xml.SetAttrib( "name", "Tractornode" );
            xml.SetAttrib( "type", "Tractor" );
        }
>>>>>>> 9291eb2fcc6ea6c1843e53809f508dbb2f828522
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
