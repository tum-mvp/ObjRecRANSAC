#include "RandomGenerator.h"
#include <ctime>

using namespace tum;

bool RandomGenerator::randomGeneratorInitialized = false;

RandomGenerator::RandomGenerator()
{
	if ( !randomGeneratorInitialized )
	{
		srand((unsigned int)time(NULL));
		randomGeneratorInitialized = true;
	}
}

RandomGenerator::~RandomGenerator()
{
}

//===================================================================================================================================
