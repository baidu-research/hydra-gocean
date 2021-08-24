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

#include "../main.h"

namespace ArgusSamples
{

ControlSettings::ControlSettings(CameraModulesEGL *module)
	: m_cameraModule(module)
	, m_deviceExposureCompensationRange(0.0f)
	, m_deviceIspDigitalGainRange(Argus::Range<float>(0.0f))
	, m_sensorExposureTimeRange(Argus::Range<uint64_t>(0))
	, m_sensorAnalogGainRange(Argus::Range<float>(0.0f))
	, m_sensorFrameRateRange(0.0f)
	, m_aeAntibandingMode(new ValidatorEnum<Argus::AeAntibandingMode>(
			s_aeAntibandingModes, sizeof(s_aeAntibandingModes)/sizeof(s_aeAntibandingModes[0])),
						Argus::AE_ANTIBANDING_MODE_AUTO)
	, m_aeLock(new ValidatorRange<bool>(false,true), true)
	, m_awbMode(new ValidatorEnum<Argus::AwbMode>(s_awbModes, 
			sizeof(s_awbModes) / sizeof(s_awbModes[0])),Argus::AWB_MODE_AUTO)
	, m_denoiseMode(new ValidatorEnum<Argus::DenoiseMode>(
			s_denoiseModes, sizeof(s_denoiseModes) / sizeof(s_denoiseModes[0])),
						Argus::DENOISE_MODE_OFF)
	, m_denoiseStrength(new ValidatorRange<float>(-1.0f, 1.0f), -1.0f)
	, m_edgeEnhanceMode(new ValidatorEnum<Argus::EdgeEnhanceMode>(
			s_edgeEnhanceModes, sizeof(s_edgeEnhanceModes) / sizeof(s_edgeEnhanceModes[0])),
						Argus::EDGE_ENHANCE_MODE_FAST)
	, m_edgeEnhanceStrength(new ValidatorRange<float>(-1.0f, 1.0f), -1.0f)
	, m_ispDigitalGainRange(new ValidatorRange<Argus::Range<float> >(&m_deviceIspDigitalGainRange),
						Argus::Range<float>(1.0f))
	, m_exposureCompensation(new ValidatorRange<float>(&m_deviceExposureCompensationRange), 0.0f)
	, m_exposureTimeRange(new ValidatorRange<Argus::Range<uint64_t> >(&m_sensorExposureTimeRange),
						Argus::Range<uint64_t>(0))
	, m_gainRange(new ValidatorRange<Argus::Range<float> >(&m_sensorAnalogGainRange),
						Argus::Range<float>(0.0f))
	, m_frameRate(new ValidatorRange<float>(&m_sensorFrameRateRange), 0.0f)
{
    PROPAGATE_ERROR_CONTINUE(ControlsInitialize());
}

bool ControlSettings::ControlsInitialize() 
{
    // Updating Control interfaces and control ranges
    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(m_cameraModule->getDevice());
    ISensorMode	*iSensorMode 		 = interface_cast<ISensorMode>(m_cameraModule->getSensorMode());
    IRequest *iRequest 			 = interface_cast<IRequest>(m_cameraModule->getRequest());
    if (!iCameraProperties || !iSensorMode || !iRequest) 
	ORIGINATE_ERROR("Cannot create Control Interfaces");

    m_iAutoControlSettings   = interface_cast<IAutoControlSettings>(iRequest->getAutoControlSettings());
    m_iSourceControlSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    m_iDenoiseSettings       = interface_cast<IDenoiseSettings>(m_cameraModule->getRequest());
    m_iEdgeEnhanceSettings   = interface_cast<IEdgeEnhanceSettings>(m_cameraModule->getRequest());
    if (!m_iAutoControlSettings || !m_iSourceControlSettings || !m_iDenoiseSettings || !m_iEdgeEnhanceSettings)
	ORIGINATE_ERROR("Failed to create Control Setting interfaces");

    Argus::Range<float> digitalGainRange = iCameraProperties->getIspDigitalGainRange();
    Argus::Range<float> deviceExposureCompensationRange =
         iCameraProperties->getExposureCompensationRange();

    Argus::Range<float> unifiedDigitalGainRange(0);
    unifiedDigitalGainRange.min() =
        std::min(m_deviceIspDigitalGainRange.get().min().min(), digitalGainRange.min());
    unifiedDigitalGainRange.max() =
        std::max(m_deviceIspDigitalGainRange.get().max().max(), digitalGainRange.max());

    Argus::Range<float> unifiedExposureCompensationRange(0);
    unifiedExposureCompensationRange.min() =
        std::min(m_deviceExposureCompensationRange.get().min(),
            deviceExposureCompensationRange.min());
    unifiedExposureCompensationRange.max() =
        std::max(m_deviceExposureCompensationRange.get().max(),
            deviceExposureCompensationRange.max());

    PROPAGATE_ERROR(m_deviceIspDigitalGainRange.set(
        Argus::Range<Argus::Range<float> >(unifiedDigitalGainRange)));
    PROPAGATE_ERROR(m_deviceExposureCompensationRange.set(
        Argus::Range<float> (unifiedExposureCompensationRange)));

    // update dependent values
    PROPAGATE_ERROR(m_ispDigitalGainRange.set(digitalGainRange));
    PROPAGATE_ERROR(m_exposureCompensation.set(0.0f));

    // set to final range
    PROPAGATE_ERROR(m_deviceIspDigitalGainRange.set(Argus::Range<Argus::Range<float> >(
        digitalGainRange, digitalGainRange)));
    PROPAGATE_ERROR(m_deviceExposureCompensationRange.set(Argus::Range<float> (
       deviceExposureCompensationRange)));

    //Updating Camera Sensor Controls
    Argus::Range<uint64_t> sensorExposureTimeRange 	= iSensorMode->getExposureTimeRange();
    Argus::Range<float> sensorAnalogGainRange(iSensorMode->getAnalogGainRange().min(),
					floor(iSensorMode->getAnalogGainRange().max()));

    Argus::Range<TimeValue> sensorFrameDurationRange(
	TimeValue::fromNSec(iSensorMode->getFrameDurationRange().min()),
	TimeValue::fromNSec(iSensorMode->getFrameDurationRange().max()));
    Argus::Range<float> sensorFrameRateRange(MIN_FPS,
	sensorFrameDurationRange.min().toCyclesPerSec());

    if (g_masterDev)
	sensorFrameRateRange.max() = DEFAULT_FPS;

    Argus::Range<uint64_t> unifiedSensorExposureTimeRange(0);
    unifiedSensorExposureTimeRange.min() = 
	std::min(m_sensorExposureTimeRange.get().min().min(), sensorExposureTimeRange.min());
    unifiedSensorExposureTimeRange.max() = 
	std::max(m_sensorExposureTimeRange.get().max().max(), sensorExposureTimeRange.max());

    Argus::Range<float> unifiedSensorAnalogGainRange(0);
    unifiedSensorAnalogGainRange.min() =
	std::min(m_sensorAnalogGainRange.get().min().min(), sensorAnalogGainRange.min());
    unifiedSensorAnalogGainRange.max() =
	floor(std::max(m_sensorAnalogGainRange.get().max().max(), sensorAnalogGainRange.max()));

    Argus::Range<float> unifiedSensorFrameRateRange(0.0f);
    unifiedSensorFrameRateRange.min() = MIN_FPS;

    unifiedSensorFrameRateRange.max() =
	std::max(m_sensorFrameRateRange.get().max(), sensorFrameRateRange.max());

    PROPAGATE_ERROR(m_sensorExposureTimeRange.set(
	Argus::Range<Argus::Range<uint64_t> >(unifiedSensorExposureTimeRange)));
    PROPAGATE_ERROR(m_exposureTimeRange.set(sensorExposureTimeRange));

    PROPAGATE_ERROR(m_sensorAnalogGainRange.set(
	Argus::Range<Argus::Range<float> >(unifiedSensorAnalogGainRange)));
    PROPAGATE_ERROR(m_gainRange.set(sensorAnalogGainRange));

    PROPAGATE_ERROR(m_sensorFrameRateRange.set(unifiedSensorFrameRateRange));
    PROPAGATE_ERROR(m_frameRate.set(sensorFrameRateRange.max()));

    PROPAGATE_ERROR(m_sensorExposureTimeRange.set(Argus::Range<Argus::Range<uint64_t> >(
        sensorExposureTimeRange, sensorExposureTimeRange)));
    PROPAGATE_ERROR(m_sensorAnalogGainRange.set(Argus::Range<Argus::Range<float> >(
        sensorAnalogGainRange, sensorAnalogGainRange)));
    PROPAGATE_ERROR(m_sensorFrameRateRange.set(sensorFrameRateRange));
    // end Updating Sensor Controls Range

    return true;
}

bool ControlSettings::SetDefaults()
{
    ISensorMode	*iSensorMode = interface_cast<ISensorMode>(m_cameraModule->getSensorMode());
    if (!iSensorMode)
	ORIGINATE_ERROR("Failed to get iSensorMode interface");

    // Set Default Control Values
    m_iSourceControlSettings->setGainRange(iSensorMode->getAnalogGainRange().min());
    m_iSourceControlSettings->setFrameDurationRange(iSensorMode->getFrameDurationRange().min());
    m_iSourceControlSettings->setExposureTimeRange(iSensorMode->getExposureTimeRange());
    m_iAutoControlSettings->setExposureCompensation(m_exposureCompensation.get());
    m_iAutoControlSettings->setAeLock(m_aeLock.get());
    m_iAutoControlSettings->setAwbMode(m_awbMode.get());
    m_iAutoControlSettings->setAeAntibandingMode(m_aeAntibandingMode.get());
    m_iDenoiseSettings->setDenoiseMode(m_denoiseMode.get());
    m_iDenoiseSettings->setDenoiseStrength(m_denoiseStrength.get());
    m_iEdgeEnhanceSettings->setEdgeEnhanceMode(m_edgeEnhanceMode.get());
    m_iEdgeEnhanceSettings->setEdgeEnhanceStrength(m_edgeEnhanceStrength.get());
    m_iAutoControlSettings->setIspDigitalGainRange(m_ispDigitalGainRange.get());

    PROPAGATE_ERROR(Start());

    return true;
}

bool ControlSettings::Start()
{
    ICaptureSession *iCaptureSession = interface_cast<Argus::ICaptureSession>(m_cameraModule->getSession());
    if(!iCaptureSession) 
	ORIGINATE_ERROR("Failed to get Capture Session");
    if(iCaptureSession->repeat(m_cameraModule->getRequest()) != Argus::STATUS_OK)
 	ORIGINATE_ERROR("Failed to submit capture request");
    return true;
}

bool ControlSettings::GetSetControls()
{
     uint32_t ControlID;
     bool err = false;

     drawHeader();
     std::cout << tabSpace() <<"Choose the Control ID Number\t" << tabSpace();
     while(!(std::cin >> ControlID)){
	std::cout << "Enter a Valid Input:" << std::endl;
	std::cin.clear();
	std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
     }
     drawHeader();

     if (!g_streamState)
	 return true;

     switch(ControlID)
     {
	case CTRL_AEANTIBANDING_MODE:
		if(userAeAntibandingMode()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated AE Antibanding Mode\t" << tabSpace() 
					<< userGetAeAntibandingMode() << std::endl;
		} else
			err = true;
		break;

	case CTRL_AE_LOCK:
		if(userAeLock()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated AE Lock\t\t\t" << tabSpace() << 
					userGetAeLock() << std::endl;
		} else
			err = true;
		break;

	case CTRL_AWB_MODE:
		if(userAwbMode()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated AWB Mode\t\t" << tabSpace() << 
					userGetAwbMode() << std::endl;
		} else
			err = true;
		break;

	case CTRL_DENOISE_MODE:
		if(userDenoiseMode()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Denoise Mode\t\t" << tabSpace() 
					<< userGetDenoiseMode() << std::endl;
		} else
			err = true;
		break;

	case CTRL_DENOISE_STRENGTH:
		if(userDenoiseStrength()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Denoise Strength\t" << tabSpace() 
					<< userGetDenoiseStrength() << std::endl;
		} else
			err = true;
		break;

	case CTRL_EDGE_ENHANCE_MODE:
		if(userEdgeEnhanceMode()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Edge Enhance Mode\t" << tabSpace() 
					<< userGetEdgeEnhanceMode() << std::endl;
		} else
			err = true;
		break;

	case CTRL_EDGE_ENHANCE_STRENGTH:
		if(userEdgeEnhanceStrength()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Edge Enhance Strength\t" << tabSpace() 
					<< userGetEdgeEnhanceStrength() << std::endl;
		} else
			err = true;
		break;

	case CTRL_ISP_GAIN_RANGE:
		if(userIspDigitalGainRange()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Isp Digital Gain Range\t" << 
					tabSpace() << userGetIspDigitalGainRange() << std::endl;
		} else
			err = true;
		break;

	case CTRL_EXPOSURE_TIME_RANGE:
		if(userExposureTimeRange()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Exposure Time Range\t" << tabSpace() 
					<< userGetExposureTimeRange() << std::endl;
		} else
			err = true;
		break;

	case CTRL_SENSOR_GAIN_RANGE:
		if(userSensorAnalogGainRage()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Sensor Gain Range\t" << 
					tabSpace() << userGetSensorAnalogGainRange() << std::endl;
		} else
			err = true;
		break;

	case CTRL_FRAME_RATE:
		if(userFrameRate()) {
		    if (g_controlState == CTRL_STATE_SET)
			std::cout << GREEN << tabSpace() << "Updated Frame Rate\t\t" << tabSpace() << 
					userGetFrameRate() << std::endl;
		} else
			err = true;
		break;

	case CTRL_EXPOSURE_COMPENSATION:
		if (userExposureCompensation()) {
		    if (g_controlState == CTRL_STATE_SET)
		    	std::cout << GREEN << tabSpace() << "Updated Exp. Compensation Value\t" <<
					tabSpace() << userGetExposureCompensation() << std::endl;
		} else
		    err = true;
		break;

	default: std::cout << RED << "Invalid Control ID / Control either not supported or wrong Control ID" << std::endl;
		 err = true;
		 break;
    }

    if (g_controlState == CTRL_STATE_SET)
	drawLine();

    if (err) {
	std::cout << RED << "Control ID : " << ControlID << " Check Valid Values" << std::endl;
	return false;
    } else 
	if (g_controlState == CTRL_STATE_SET)
	    PROPAGATE_ERROR(Start());
	return true;
}

bool ControlSettings::userAeAntibandingMode()
{
    std::string userString;
    std::cout << tabSpace() <<"Current AE Antibanding Mode\t" << tabSpace() << BLUE << 
					m_aeAntibandingMode.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_aeAntibandingMode.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_aeAntibandingMode.setFromString(userString.c_str())) {
	if(m_iAutoControlSettings->setAeAntibandingMode(m_aeAntibandingMode) == Argus::STATUS_OK)
	    return true;
	else 
	    return false;
    } else 
	return false;
}

bool ControlSettings::userAeLock()
{
    std::string userString;
    std::cout << tabSpace() << "Current AE Lock\t\t\t" << tabSpace() << BLUE << m_aeLock.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_aeLock.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_aeLock.setFromString(userString.c_str())) {
	if(m_iAutoControlSettings->setAeLock(m_aeLock) == Argus::STATUS_OK)
	    return true;
	else 
	    return false;
    } else 
	return false;
}

bool ControlSettings::userAwbMode()
{
    std::string userString;
    std::cout << tabSpace() << "Current AWB Mode\t\t" << tabSpace() << BLUE << m_awbMode.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_awbMode.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_awbMode.setFromString(userString.c_str())) {
	if(m_iAutoControlSettings->setAwbMode(m_awbMode) == STATUS_OK)
	    return true;
	else
	    return false;
    } else 
	return false;
}

bool ControlSettings::userDenoiseMode()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current Denoise Mode\t\t" << tabSpace() << BLUE << m_denoiseMode.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_denoiseMode.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_denoiseMode.setFromString(userString.c_str())) {
	if(m_iDenoiseSettings->setDenoiseMode(m_denoiseMode) == STATUS_OK)
	    return true;
	else
	    return false;
    } else 
	return false;
}

bool ControlSettings::userDenoiseStrength()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current Denoise Strength\t" << tabSpace() << BLUE << m_denoiseStrength.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_denoiseStrength.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_denoiseStrength.setFromString(userString.c_str())) {
	if(m_iDenoiseSettings->setDenoiseStrength(m_denoiseStrength) == STATUS_OK)
	    return true;
	else
	    return false;
    } else 
	return false;
}

bool ControlSettings::userEdgeEnhanceMode()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current Edge Enhance Mode\t" << tabSpace() << BLUE << m_edgeEnhanceMode.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_edgeEnhanceMode.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_edgeEnhanceMode.setFromString(userString.c_str())) {
	if(m_iEdgeEnhanceSettings->setEdgeEnhanceMode(m_edgeEnhanceMode) == STATUS_OK)
	    return true;
	else
	    return false;
    } else 
	return false;
}

bool ControlSettings::userEdgeEnhanceStrength()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current Edge Enhance Strength\t" << tabSpace() << BLUE << m_edgeEnhanceStrength.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_edgeEnhanceStrength.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_edgeEnhanceStrength.setFromString(userString.c_str())) {
	if(m_iEdgeEnhanceSettings->setEdgeEnhanceStrength(m_edgeEnhanceStrength) == STATUS_OK)
	    return true;
	else 
	    return false;
    } else 
	return false;
}

bool ControlSettings::userIspDigitalGainRange()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current ISP Digital Gain Range\t" << tabSpace() << BLUE << m_ispDigitalGainRange.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_ispDigitalGainRange.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    Argus::Range<float> Range;
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_ispDigitalGainRange.convertFromString(userString.c_str(),Range)) {
	if (Range.min() > Range.max())
		Range.swap();
	m_ispDigitalGainRange.set(Range);
	if(m_iAutoControlSettings->setIspDigitalGainRange(m_ispDigitalGainRange) == STATUS_OK)
		return true;
	else
		return false;
    } else 
	return false;
}

bool ControlSettings::userExposureTimeRange()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current Exposure Time Range\t" << tabSpace() << BLUE << m_exposureTimeRange.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_exposureTimeRange.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;

    Argus::Range<uint64_t> Range; 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_exposureTimeRange.convertFromString(userString.c_str(),Range)) {
	if (Range.min() > Range.max())
		Range.swap();
	m_exposureTimeRange.set(Range);
	if(m_iSourceControlSettings->setExposureTimeRange(m_exposureTimeRange.get()) == STATUS_OK) 
	    return true;
	else 
	    return false;
    } else 
	return false;
}

bool ControlSettings::userSensorAnalogGainRage()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current Sensor Gain Range\t" << tabSpace() << BLUE << m_gainRange.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_gainRange.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;

    Argus::Range<float> Range; 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if(m_gainRange.convertFromString(userString.c_str(),Range)) {
	if (Range.min() > Range.max())
		Range.swap();
	m_gainRange.set(Range);
	if(m_iSourceControlSettings->setGainRange(m_gainRange) == STATUS_OK)
	    return true;
	else
	    return false;
    } else 
	return false;
}

bool ControlSettings::userFrameRate()
{
    Argus::Range<uint64_t> frameDurationRange(0);
    std::string userString;

    std::cout << tabSpace() << YELLOW <<"Current Frame Rate\t\t" << tabSpace() << BLUE << m_frameRate.toString() << std::endl;
    drawLine();

    if (g_masterDev && g_deviceID != 0) {
    	std::cout << tabSpace() << GREEN << "Frame Rate Control Disabled in Slave Device\t" << std::endl;
    	drawLine();
	return false;
    }
 
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_frameRate.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;

    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();

    ISensorMode	*iSensorMode = interface_cast<ISensorMode>(m_cameraModule->getSensorMode());

    if(m_frameRate.setFromString(userString.c_str())) {
	if(m_frameRate.get() == 0.0f) 
	    frameDurationRange = iSensorMode->getFrameDurationRange();
	else
	    frameDurationRange = TimeValue::fromCycelsPerSec(m_frameRate.get()).toNSec();

	// Limit Framerate to 30fps for Synchronization
	if (g_masterDev && (frameDurationRange.min() < (1e9/DEFAULT_FPS))) {
	    frameDurationRange = 1e9/DEFAULT_FPS;
	    m_frameRate.set(DEFAULT_FPS);
	}
	g_masterFrameRate = m_frameRate.get();
	g_frameRateChange = true;

	if(m_iSourceControlSettings->setFrameDurationRange(frameDurationRange) == STATUS_OK)
	    return true;
	else
	    return false;
    } else 
	return false;
}

bool ControlSettings::userSetSlaveFrameRate()
{
    Range<uint64_t> frameDurationRange;
    ISensorMode	*iSensorMode = interface_cast<ISensorMode>(m_cameraModule->getSensorMode());
    
    if (!g_masterDev)
	return true;

    if (g_frameRateChange) {
	m_frameRate.set(g_masterFrameRate);
	if (m_frameRate.get() == 0.0f)
	    frameDurationRange = iSensorMode->getFrameDurationRange();
	else
	    frameDurationRange = TimeValue::fromCycelsPerSec(m_frameRate.get()).toNSec();

	if (m_iSourceControlSettings->setFrameDurationRange(frameDurationRange) == STATUS_OK) {
	    PROPAGATE_ERROR(Start());
	    return true;

	} else
	    return false;
    }
    return true;
}

bool ControlSettings::userExposureCompensation()
{
    std::string userString;
    std::cout << tabSpace() << YELLOW <<"Current Exp. Compensation Value\t" << tabSpace() << BLUE << m_exposureCompensation.toString() << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "Valid Control Values/Range\t" << tabSpace() << GREEN << 
					m_exposureCompensation.getValidValuesMessage() << std::endl;
    drawLine();

    if (g_controlState == CTRL_STATE_GET)
	return true;
 
    std::cout << tabSpace() << BLUE << "Enter User Control Value\t" << tabSpace();
    std::cin >> userString;
    drawLine();
    if (m_exposureCompensation.setFromString(userString.c_str())) {
	if (m_iAutoControlSettings->setExposureCompensation(m_exposureCompensation) == STATUS_OK)
	    return true;
	else
	    return false;
    } else
	return false;
}

void ControlSettings::ListControls()
{
    mainMenu("Printing Sensor and ISP Controls with Range/Values");
    std::cout << tabSpace() << YELLOW << "CTRL ID\t" << tabSpace() << YELLOW << "Control Name\t\t" << 
		tabSpace() << YELLOW << "Valid Values\t\t\t\t|" << std::endl;
    drawLine();
    std::cout << tabSpace() << GREEN << "1.\t" << tabSpace() << "AE AntiBanding\t\t" << tabSpace() << 
		m_aeAntibandingMode.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "2.\t" << tabSpace() << "AE Lock\t\t\t" << tabSpace() << 
		m_aeLock.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "3.\t" << tabSpace() << "AWB Mode\t\t" << tabSpace() << 
		m_awbMode.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "4.\t" << tabSpace() << "Denoise Mode\t\t" << tabSpace() << 
		m_denoiseMode.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "5.\t" << tabSpace() << "Denoise Strength\t" << tabSpace() << 
		m_denoiseStrength.getValidValuesMessage() <<std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "6.\t" << tabSpace() << "EdgeEnhance Mode\t" << tabSpace() << 
		m_edgeEnhanceMode.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "7.\t" << tabSpace() << "EdgeEnhance Strength\t" << tabSpace() << 
		m_edgeEnhanceStrength.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "8.\t" << tabSpace() << "ISP Gain Range\t\t" << tabSpace() << 
		m_ispDigitalGainRange.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "9.\t" << tabSpace() << "Exposure Compensation\t" << tabSpace() << 
		m_exposureCompensation.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "10.\t" << tabSpace() << "Exposure Time Range\t" << tabSpace() << 
		m_exposureTimeRange.getValidValuesMessage() << std::endl;
    drawLine();

    std::cout << tabSpace() << GREEN << "11.\t" << tabSpace() << "Sensor Gain Range\t" << tabSpace() << 
		m_gainRange.getValidValuesMessage() << std::endl;
    drawLine();
	
    std::cout << tabSpace() << GREEN << "12.\t" << tabSpace() << "Sensor Frame Rate\t" << tabSpace() <<
		m_frameRate.getValidValuesMessage() << std::endl;
    drawLine();
    return;
}

};// NameSpace ArgusSamples
