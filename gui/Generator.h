/**
 * Generator header file. Converts xml documents to world file
 */
#include "tinyxml2.h"
#include <string>
#include <fstream>

using namespace tinyxml2;
using namespace std;

class Generator
{
	public:
		Generator(string inputName, string outputName);
		void loadWorld();
		void loadOrchard();
		void loadRobots();
		void loadPeople();
		void loadAnimals();
        void loadTallWeeds();
		void write();

	private:
		string inputName;
		string outputName;
		XMLDocument doc;
		XMLElement* rootElement;
		ofstream outfile;

		// static const variables
		// distance between trunk/pole and beacon. x coord
		int const static SEPARATION = 2;

		// height and width of bitmap png
		int const static WIDTH = 950;
		int const static HEIGHT = 550;

};
