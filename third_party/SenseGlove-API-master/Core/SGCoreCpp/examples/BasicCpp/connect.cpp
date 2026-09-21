/*
A console application demonstrating the intended use of a SenseGlove interface.
Used to compile the programs in the SGCoreCpp/test folder.
*/

#include <iostream> //Output to console

#include <thread>  //To pause the main() while vibrating
#include <chrono>  //To pause the thread for std::chrono::seconds

#include "incl/Library.h" //Contains version information on SGCore / SGConnect Libraries
#include "incl/SenseCom.h" //Functions to check scanning process - and to start it if need be.

#include "incl/HapticGlove.h" //Haptic Glove Interfacing
#include "incl/Tracking.h" //To calculate wrist location based on fixed hardware offsets.
#include "incl/SG_FFBCmd.h" //force-feedback command(s)
#include "incl/SG_BuzzCmd.h" //vibration command(s)

int main()
{
	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Checking the Library

	// Displaying Library information - Useful to know when asking for any kind of support
	std::cout << "Testing " + SGCore::Library::Version() + ", compiled for " + SGCore::Library::BackEndVersion();
	if (SGCore::Library::GetBackEndType() == SGCore::BackEndType::SharedMemory) // By default, your library will be compiled to use Shared Memory via the SGConnect library
    {
		std::cout << " using " + SGCore::Library::SGConnectVersion(); //If you replace SGConnect.dll, this will give you its current version number.
    }
	std::cout << std::endl;
	std::cout << "The source code for this program is located in the SGCoreCs/test/ folder." << std::endl;
	std::cout << "=========================================================================" << std::endl;


	//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
	// Ensuring connectivity

	// Connecting to SenseGlove devices is done in a separate "Connection Process" - contained in the SGConnect library.
	// We can test if this Connection Process is running on this PC. Usually, it runs inside SenseCom.
	// It's good practice to start this process if your user has not sone so yet.
	{
		bool connectionsActive = SGCore::SenseCom::ScanningActive(); //returns true if SenseCom (or another program) has started the SenseGlove Communications Process.
		if (!connectionsActive) // If this process is not running yet, we can "Force-Start" SenseCom. Provided it has run on this PC at least once.
		{
			std::cout << "SenseCom is not yet running. Without it, we cannot connect to SenseGlove devices." << std::endl;
			bool startedSenseCom = SGCore::SenseCom::StartupSenseCom(); //Returns true if the process was started.
			if (startedSenseCom)
			{
				std::cout << "Successfully started SenseCom. It will take a few seconds to connect..." << std::endl;
				connectionsActive = SGCore::SenseCom::ScanningActive(); //this will return false immedeately after you called StartupSenseCom(). Because the program has yet to initialize.
																		// Even if SenseCom started and the connections process is active, there's no guarantee that the user has turned their device(s) on. More on that later.
			}
			else // If StartupSenseCom() returns false, you've either never run SenseCom, or it is already running. But at that point, the ScanningActive() should have returned true. 
			{
				std::cout << "Could not Start the SenseCom process. This is not yet implemented in our C++ API.";
			}
			std::cout << std::endl;
		}
	}
}