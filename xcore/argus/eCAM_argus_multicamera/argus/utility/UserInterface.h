/*
 * Copyright (c) 2016-2018, e-con Systems India Pvt. Ltd CORPORATION. All rights reserved.
 * Author : Waiss Kharni <waisskharni.sm@e-consystems.com>
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
#ifndef USERINTER_H
#define USERINTER_H

#include <Argus/Argus.h>
#include "CameraModuleEGL.h"

#define NUM_CAMERAS 6

using namespace Argus;

void drawLine();

void mainMenu(const char *userString);

void drawHeader();

std::string tabSpace();

namespace ArgusSamples
{

class CommandInterface 
{
public:
	explicit CommandInterface(std::vector<CameraModulesEGL*> CamMods) :
		m_cameraMods(CamMods),
		StreamCount(CamMods.size())
	{
	}

	virtual ~CommandInterface();
	
	virtual bool Initialize();
	virtual bool UserCommandExecute();
	virtual bool Shutdown();

	ControlSettings* getControlInterface(uint32_t DeviceID)
	{
	     return m_controlSettings[DeviceID].get();
	}

private:
	std::vector<CameraModulesEGL*> m_cameraMods;
	uint32_t choice, deviceID, StreamCount;
	UniqueObj<ControlSettings> m_controlSettings[NUM_CAMERAS];
};

}; //Namespace ArgusSamples

#endif
