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

#include <Argus/Argus.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#define MAX_CAMS 6
class MasterSlaveMode
{
public:
	MasterSlaveMode(bool MasterSlaveMode)
		: m_masterSlaveMode(MasterSlaveMode)
		, m_initialized(false)
	{
	}

	~MasterSlaveMode()
	{
	    for (uint32_t i = 0; i<m_numCameras; i++)
		if (camFd[i])
		    close(camFd[i]);
	}

	enum {
	    SET_MASTER_MODE = 0x3B,
	    DEFAULT_ALLMASTER = 0x00,
	};

	bool changeMasterSlaveMode(bool Mode);

	bool initialize(uint32_t num_Cameras);

	bool queryMasterSlaveControl(bool PrintEnable, uint32_t num_Cameras);

	bool setMasterSlaveMode();

private:
	uint32_t 		camFd[MAX_CAMS], m_numCameras;
	struct v4l2_queryctrl	m_controls;
	struct v4l2_control	m_userControl;
	bool			m_ctrlFlag[MAX_CAMS],m_deviceFlag[MAX_CAMS], m_masterSlaveMode, m_initialized;
};
