/**
 * Generator header file. Converts xml documents to world file
 */
#include "tinyxml2.h"
#include <ofstream>
#include <string>

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
		void write();

	private:
		string inputName;
		string outputName;
		XMLDocument doc;
		XMLElement* rootElement;
		ofstream outfile;
};
