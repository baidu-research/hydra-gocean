/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../main.h"

using namespace Argus;


void drawLine()
{
	std::cout << GREEN << ":-----------------------------------------------------------------------------------------------:" << RESET << std::endl;
}

void mainMenu(const char *userString)
{
	std::cout << std::endl;
	std::cout << GREEN << ":===============================================================================================:" << RESET << std::endl;
	std::cout << YELLOW << "|			" << userString << "			|" << RESET << std::endl;
	std::cout << GREEN << ":===============================================================================================:" << RESET << std::endl;
}

void drawHeader()
{
	std::cout << GREEN << ":===============================================================================================:" << RESET << std::endl;
}
	
std::string tabSpace()
{
	std::ostringstream String;
	String << GREEN << "|\t" << RESET;
	return String.str();
}

namespace ArgusSamples
{

CommandInterface::~CommandInterface()
{
}

bool CommandInterface::Initialize()
{
    for (uint32_t i = 0; i < StreamCount; i++)
	m_controlSettings[i].reset(new ControlSettings(m_cameraMods[i]));

    return true;
}

bool CommandInterface::UserCommandExecute()
{
    uint32_t choice, captureChoice;
    bool ret = true, ModifyDevice = false;
    g_deviceID = 0;
    signal(SIGINT, signal_handler);

    printf("\n");
    drawHeader();
    std::cout << tabSpace() << "Choice ID\t" << tabSpace() << "Description" << std::endl;
    drawHeader();
    std::cout << tabSpace() << "1.\t\t" << tabSpace() << "Modify/Select Device " << std::endl;
    drawLine();
    std::cout << tabSpace() << "2.\t\t" << tabSpace() << "Burst Capture " << RESET << std::endl;
    drawLine();
    if (interface_cast<ISensorMode>(m_cameraMods[g_deviceID]->getSensorMode())->getFrameDurationRange().min() >= 33333333) {
	std::cout << tabSpace() << "3.\t\t" << tabSpace() << "Burst Record " << RESET << std::endl;
	drawLine();
    	std::cout << tabSpace() << "4.\t\t" << tabSpace() << "Exit Application" << std::endl;
    	drawLine();
    } else {
    	std::cout << tabSpace() << "3.\t\t" << tabSpace() << "Exit Application" << std::endl;
    	drawLine();
    }
    std::cout << WHITE <<"Enter your choice\t" << tabSpace();
    while(!(std::cin >> choice)){
	std::cout << "Enter a Valid Input\t" << tabSpace();
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }

    drawLine();
    printf("\n");
    
    switch(choice)
    {
	case 1:	ModifyDevice = true;
		break;

	case 2:	drawLine();
		std::cout << tabSpace() << "1.\t\t" << tabSpace() << "JPEG Capture" << std::endl;
		drawLine();
		std::cout << tabSpace() << "2.\t\t" << tabSpace() << "RAW Capture" << std::endl;
		drawLine();
    		std::cout << WHITE <<"Enter your choice\t" << tabSpace();
		while(!(std::cin >> captureChoice) || ret) {
		    if (captureChoice == 1 || captureChoice == 2) {
			ret = false;
			break;
		    }
		    std::cout << tabSpace() << "Enter a Valid Input :\t" << tabSpace();
		    std::cin.clear();
		    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		}
		g_captureType = captureChoice == 1 ? CAPTURE_TYPE_JPEG : CAPTURE_TYPE_RAW;
		g_burstCaptureFlag = true;
		while(g_burstCaptureFlag && g_streamState);
		break;

	case 3: if (interface_cast<ISensorMode>(m_cameraMods[g_deviceID]->getSensorMode())->getFrameDurationRange().min() >= 33333333) { 
		    g_captureType = CAPTURE_TYPE_H264;
		    g_burstRecordFlag = true;
		    while(g_burstRecordFlag && g_streamState);
		    drawLine();
		    std::cout << tabSpace() << "1.\t" << tabSpace() << "Stop recording video" << std::endl;
		    drawLine();
		    while(!(std::cin >> captureChoice) || ret) {
		    	if (captureChoice == 1) {
		    	    ret = false;
			    break;
		    	}
		    	std::cout << tabSpace() << "Enter a Valid Input :\t" << tabSpace();
		    	std::cin.clear();
		    	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		    }
		    g_burstStopRecord = true;
		    while(g_burstStopRecord && g_streamState);
		} else {
		    g_streamState = 0;
		    g_calibrate = false;
		}
		break;

	case 4: if (interface_cast<ISensorMode>(m_cameraMods[g_deviceID]->getSensorMode())->getFrameDurationRange().min() >= 33333333) {
		    g_streamState = 0;
		    g_calibrate = false;
		} else
		    std::cout << "Enter a Valid Choice" << std::endl;
		break;

	default: std::cout << "Enter a Valid Choice" << std::endl;
    }

    if (ModifyDevice) {

	drawHeader();
	std::cout << tabSpace() << BLUE << "No. of. Devices Launched\t" << RESET;
	std::cout << tabSpace() << YELLOW << StreamCount <<"\t"<< tabSpace() << 
		    "Range [ 0 - "<< StreamCount-1 << " ]\n";
	drawHeader();
	std::cout << tabSpace() << "Choose Device ID\t\t" << tabSpace();
	while(!(std::cin >> g_deviceID)){
	    std::cout << "Enter a Valid Input:" << std::endl;
	    std::cin.clear();
	    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}

	if (g_deviceID >= StreamCount) {
	    drawLine();
	    std::cout << tabSpace() << RED << "Device ID "<< g_deviceID << 
		    "not in Range. Using Default Device!" << std::endl;
	    g_deviceID = 0;
	}

	printf("\n");
	drawHeader();
	std::cout << tabSpace() << "Choice ID\t" << tabSpace() << "Description" << std::endl;
	drawHeader();
	std::cout << tabSpace() << "1.\t\t" << tabSpace() << "List Controls" << std::endl;
	drawLine();
	std::cout << tabSpace() << "2.\t\t" << tabSpace() << "Get Control Value" << std::endl;
	drawLine();
	std::cout << tabSpace() << "3.\t\t" << tabSpace() << "Set Control Value" << std::endl;
	drawLine();
	std::cout << tabSpace() << "4.\t\t" << tabSpace() << "Get Stream Metadata" << std::endl;
	drawLine();
	std::cout << tabSpace() << "5.\t\t" << tabSpace() << "Image Capture" << std::endl;
	drawLine();
	std::cout << tabSpace() << "6.\t\t" << tabSpace() << "Video Record" << std::endl;
	drawLine();
	std::cout << tabSpace() << "7.\t\t" << tabSpace() << "Main Menu" << std::endl;
	drawLine();

	std::cout << WHITE <<"Enter your choice\t" << tabSpace();
	while(!(std::cin >> choice)){
	    std::cout << "Enter a Valid Input:" << std::endl;
	    std::cin.clear();
	    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
	}
	drawLine();

	switch(choice)
	{
	    case 1: m_controlSettings[g_deviceID].get()->ListControls();
		    break;

	    case 2: m_controlSettings[g_deviceID].get()->ListControls();
		    g_controlState = CTRL_STATE_GET;
		    m_controlSettings[g_deviceID].get()->GetSetControls();
		    break;

	    case 3: m_controlSettings[g_deviceID].get()->ListControls();
		    g_controlState = CTRL_STATE_SET;
		    m_controlSettings[g_deviceID].get()->GetSetControls();
		    break;

	    case 4: g_getStreamMetadata = true;
		    while(g_getStreamMetadata && g_streamState);
		    break;

	    case 5: std::cout << tabSpace() << "1.\t\t" << tabSpace() << "JPEG Capture" << std::endl;
		    drawLine();
		    std::cout << tabSpace() << "2.\t\t" << tabSpace() << "RAW Capture" << std::endl;
		    drawLine();
    		    std::cout << WHITE <<"Enter your choice\t" << tabSpace();
		    while(!(std::cin >> captureChoice) || ret) {
		    	if (captureChoice == 1 || captureChoice == 2) {
				ret = false;
				break;
			    }
			    std::cout << tabSpace() << "Enter a Valid Input :\t" << tabSpace();
			    std::cin.clear();
			    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		    }
 
		    g_captureType = captureChoice == 1 ? CAPTURE_TYPE_JPEG : CAPTURE_TYPE_RAW;
		    g_captureState = true;
		    while(g_captureState && g_streamState);
		    break;

	    case 6: std::cout << tabSpace() << "1.\t\t" << tabSpace() << "RAW Frame Dump" << std::endl;
		    drawLine();
		    std::cout << tabSpace() << "2.\t\t" << tabSpace() << "H264 Encoded" << std::endl;
		    drawLine();
    		    std::cout << WHITE <<"Enter your choice\t" << tabSpace();
		    while(!(std::cin >> captureChoice) || ret) {
		    	if (captureChoice == 1 || captureChoice == 2) {
				ret = false;
				break;
			    }
			    std::cout << tabSpace() << "Enter a Valid Input :\t" << tabSpace();
			    std::cin.clear();
			    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
		    }
 
		    g_captureType = captureChoice == 1 ? CAPTURE_TYPE_RAW : CAPTURE_TYPE_H264;

		    // Wait for Video Record pipeline to be created
	    	    g_videoRecordFlag = true;
		    while(g_videoRecordFlag && g_streamState);

		    if (g_captureType == CAPTURE_TYPE_H264) {
			drawLine();
			std::cout << tabSpace() << "1.\t" << tabSpace() << "Stop recording video" << std::endl;
			drawLine();
			while(!(std::cin >> captureChoice) || ret) {
			    if (captureChoice == 1) {
			         ret = false;
				 break;
			    }
			    std::cout << tabSpace() << "Enter a Valid Input :\t" << tabSpace();
			    std::cin.clear();
			    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
			}
		    } else
			usleep(5000000);

		    g_videoStopRecord = true;
		    while(g_videoStopRecord && g_streamState);
		    break;

	    case 7: break;

	    default: std::cout << tabSpace() << "Enter a valid Choice" << std::endl;
	}
    }
    if (g_frameRateChange && g_masterDev) {
	for (uint32_t dev = 1; dev < StreamCount; dev++)
	    PROPAGATE_ERROR(m_controlSettings[dev].get()->userSetSlaveFrameRate());

	g_frameRateChange = false;
    }
    return true;
}

bool CommandInterface::Shutdown()
{
    for (uint32_t i = 0; i < StreamCount; i++)
	m_controlSettings[i].reset();

	return true;
}

}; //namespace ArgusSamples
