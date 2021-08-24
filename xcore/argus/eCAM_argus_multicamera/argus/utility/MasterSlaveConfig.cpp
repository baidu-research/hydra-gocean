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

#define TEGRA_CAMERA_MASTER_CTRL 0x009a2015
#define DEV_NODE "/dev/video"

bool MasterSlaveMode::changeMasterSlaveMode(bool mode)
{
	m_masterSlaveMode = mode;
	return true;
}
bool MasterSlaveMode::initialize(uint32_t num_Cameras)
{
	if (m_initialized)
	    return true;

	m_numCameras = num_Cameras;
	for (int i = 0; i < m_numCameras; i++) {
		std::ostringstream fileID;
		fileID << DEV_NODE << i;
		m_deviceFlag[i] = false;
		m_ctrlFlag[i] = false;

		camFd[i] = open(fileID.str().c_str(),O_RDWR);
		if (!camFd[i]) {
			m_deviceFlag[i] = false;
			drawLine();
			std::cout << tabSpace() << "Device " << RED << i << RESET " not found" << std::endl;
			return false;
		}
		m_deviceFlag[i] = true;
	}
	m_initialized = true;
	return true;
}

bool MasterSlaveMode::queryMasterSlaveControl(bool PrintEnable, uint32_t num_Cameras)
{
	struct v4l2_queryctrl m_controls;
	uint8_t MasterFlag;

	if (!m_initialized)
	    PROPAGATE_ERROR(initialize(num_Cameras));

	if (m_masterSlaveMode) 
		MasterFlag = SET_MASTER_MODE;
	else
		MasterFlag = DEFAULT_ALLMASTER;

	// Finding the control
	for (int i = 0; i < m_numCameras; i++) {
		std::ostringstream fileID;
		fileID << DEV_NODE << i;
		m_controls.id = V4L2_CTRL_FLAG_NEXT_CTRL;
		while(ioctl(camFd[i], VIDIOC_QUERYCTRL, &m_controls) == 0) {
			if(m_controls.id == TEGRA_CAMERA_MASTER_CTRL) {
				m_ctrlFlag[i] = true;
				if (PrintEnable) {
				    MasterFlag & 0x01 ? std::cout << WHITE << tabSpace() << "Slave\t\t" : std::cout << WHITE << tabSpace() << "Master\t\t";
				    std::cout << tabSpace() << GREEN << fileID.str() << RESET << std::endl;
				    MasterFlag = MasterFlag >> 1;
				    drawLine();
				    break;
				}
			}
			m_controls.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
		}
		if(!m_ctrlFlag[i]) {
		    m_initialized = false;
		    return false;
		}
	}
	return true;
}

bool MasterSlaveMode::setMasterSlaveMode()
{
	uint8_t MasterFlag;
	if (m_masterSlaveMode) 
		MasterFlag = SET_MASTER_MODE;
	else
		MasterFlag = DEFAULT_ALLMASTER;

	for (int i=0; i< m_numCameras; i++) {
		m_userControl.id = TEGRA_CAMERA_MASTER_CTRL;
		m_userControl.value = MasterFlag & 0x01;
		if(m_deviceFlag[i] && m_ctrlFlag[i]) {
			if(ioctl(camFd[i], VIDIOC_S_CTRL, &m_userControl) == -1)
				std::cout << tabSpace() << "Unable to set Device " << RED << i << " to : " << m_userControl.value << RESET << std::endl;

			close(camFd[i]);
		}
		m_deviceFlag[i] = false;
		m_ctrlFlag[i] = false;
		MasterFlag = MasterFlag >> 1;
	}
	m_initialized = false;
	return true;
} 


