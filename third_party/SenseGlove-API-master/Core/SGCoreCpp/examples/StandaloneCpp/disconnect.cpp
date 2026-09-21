// SGCoreCpp_Standalone.cpp : Example to show how to build a SenseGlove project that's independent of the SenseCom program.
//

#include <iostream>
#include "incl/SGConnect.h"
#include "incl/DeviceList.h"
#include "incl/SenseGlove.h"
#include "incl/Library.h"

#include <thread>
#include <chrono> //wait untill we find SenseGloves

int main(bool scanActive)
{
    //---------------------------------------------------------------------------
	// SGConnect Cleanup - Parts taken from the SGConnect example code.
	std::cout << std::endl;
	if (!scanActive)
	{
		int disposeCode = SGConnect::Dispose();
		if (disposeCode > 0)
		{
			std::cout << "Succesfully cleaned up SGConnect resources: (DisposeCode = " << std::to_string(disposeCode) << ")" << std::endl;
			std::cout << "It's now safe to exit the program. Press return to end." << std::endl;
		}
		else
		{
			std::cout << "Unable to properly dispose of SGConnect resources: (DisposeCode = " << std::to_string(disposeCode) << ")." << std::endl;
			std::cout << "Fortunately, closing this process will cause them to go out of scope and be destroyed either way." << std::endl;
		}
	}
	else
	{
		std::cout << "Since this process did not start SGConnect, we don't need to clean anything up." << std::endl;
		std::cout << "We can call SGConnect::Dispose() safely, but this will not stop the original process." << std::endl;
		int disposeVal = SGConnect::Dispose();
		std::cout << "The SGConnect::Dispose() will return a value that is not 1, to let us know nothing was Disposed of: " << std::to_string(disposeVal) << std::endl;
		std::cout << "If you wish to dispose of the already running process, you should call SGConnect::Dispose() from the program that originally called SGConnect::Init()" << std::endl;
	}

	while (std::cin.get() != '\n') {} //Wait for the user to confirm before exiting.
}