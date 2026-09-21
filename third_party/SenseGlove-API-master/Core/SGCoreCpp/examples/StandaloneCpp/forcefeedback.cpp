#include <iostream>
#include "incl/SGConnect.h"
#include "incl/DeviceList.h"
#include "incl/SenseGlove.h"
#include "incl/Library.h"

#include <thread>
#include <chrono> //wait untill we find SenseGloves
//int a, int b, int c, int d, int e
int main(int a, int b, int c, int d, int e)
{
    std::shared_ptr<SGCore::HapticGlove> glove;
    if (SGCore::HapticGlove::GetGlove(glove))
		{
            SGCore::Haptics::SG_FFBCmd ffb(a, b, c, d, e);
            glove->SendHaptics(ffb);
    }
}