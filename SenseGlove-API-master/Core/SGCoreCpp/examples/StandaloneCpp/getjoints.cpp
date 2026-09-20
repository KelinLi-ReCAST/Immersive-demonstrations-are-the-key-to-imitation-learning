// SGCoreCpp_Standalone.cpp : Example to show how to build a SenseGlove project that's independent of the SenseCom program.
//

#include <iostream>
#include "incl/SGConnect.h"
#include "incl/DeviceList.h"
#include "incl/SenseGlove.h"
#include "incl/Library.h"

#include <thread>
#include <chrono> //wait untill we find SenseGloves
using std::string;

extern "C"{
 const char *getjoints()
//---------------------------------------------------------------------------
	// SGCoreCpp  - Parts taken from the SGCoreCpp example code.

	{
		// For a more practical example: Let's assume this is 1 frame of your simulation.
		// You've defined which glove you want to use (left/right) - in this case, let's use the same side as our testGlove.
        int cnt_index = 0, cnt_int = 0;
	    int joints[21];

		static string s="";

		std::shared_ptr<SGCore::HapticGlove> glove;
		if (SGCore::HapticGlove::GetGlove(glove))
		{
		  
			//Step 1: Hand Pose

			// We want to get the pose of the hand - to animate a virtual model
			SGCore::HandPose handPose; //The handPose class contains all data you'll need to animate a virtual hand
			if (glove->GetHandPose(handPose)) //returns the HandPose based on the latest device data, using the latest Profile and the default HandGeometry
			{
                for (int i = 0; handPose.ToString()[i] != '\0'; ++i) //当a数组元素不为结束符时.遍历字符串a.
     			{
         			if (handPose.ToString()[i] >= '0'&& handPose.ToString()[i] <= '9') //如果是数字.
         			{
					    if (handPose.ToString()[i+1] >= '0'&& handPose.ToString()[i+1] <= '9')
						{
							if (handPose.ToString()[i+2] >= '0'&& handPose.ToString()[i+2] <= '9')
							{
             			    	cnt_int = (handPose.ToString()[i] - '0')*100 + (handPose.ToString()[i] - '0')*10 + handPose.ToString()[i+1] - '0';
								if (handPose.ToString()[i-1]=='-')
								{
									cnt_int = -cnt_int;
								}
								i = i+2;	
							}
							else
							{
                                cnt_int = (handPose.ToString()[i] - '0')*10 + handPose.ToString()[i+1] - '0';
								if (handPose.ToString()[i-1]=='-')
								{
									cnt_int = -cnt_int;
								}
								i = i+1;
							}
						}
						else
						{
                            cnt_int = handPose.ToString()[i] - '0';
							if (handPose.ToString()[i-1]=='-')
								{
									cnt_int = -cnt_int;
								}
						}
					joints[cnt_index] = cnt_int;
                    cnt_index += 1;	
        		    }
     			}

			}
		     
        }
		s.erase (s.begin(), s.end());
		for (int i = 0; i<21; ++i)
		{
			s = s+std::to_string(joints[i])+",";
		}
		
		return s.c_str();
    }
}