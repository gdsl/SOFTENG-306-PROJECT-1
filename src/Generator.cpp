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
	outfile << "include \"orchard_models.inc" << endl << endl;

	// load bitmap
	// Values currently HARD CODED
	outfile << "# load an environment bitmap" << endl;
	outfile << "floorplan" << endl;
	outfile << "(" << endl;
	outfile << "\tname \"Orchard\"" << endl;
	outfile << "\tsize [95 55 0.1]" << endl;
	outfile << "\tpose [0.000 0.000 0.000 0.000]" << endl;
	outfile << "\tbitmap \"bitmaps/orchard.png\"" << endl;
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
 * Loads orchard environment
 */
void Generator::loadOrchard()
{
	XMLElement* models = rootElement -> FirstChildElement("models");

	// orchard element
	XMLElement* orchard = models -> FirstChildElement("orchard");

	// initialise variables
	int rowCount, rowLength, rowWidth, trunkPoleSpacing, trunkHeight;

	XMLElement* row_count = orchard -> FirstChildElement("row_count");
	rowCount = atoi(row_count->GetText());

	XMLElement* row_length = orchard -> FirstChildElement("row_length");
	rowLength = atoi(row_length->GetText());

	XMLElement* row_width = orchard -> FirstChildElement("row_width");
	rowWidth = atoi(row_width->GetText());

	XMLElement* trunk_pole_spacing = orchard -> FirstChildElement("trunk_pole_spacing");
	trunkPoleSpacing = atoi(trunk_pole_spacing->GetText());

	XMLElement* trunk_height = orchard -> FirstChildElement("trunk_height");
	trunkHeight = atoi(trunk_height->GetText());

	// TODO
}

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
