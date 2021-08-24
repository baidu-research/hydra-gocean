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
#include "ProcessFrameThread.h"

namespace ArgusSamples
{

ProcessFrameThread::~ProcessFrameThread()
{
    if (m_compositedFrame)
        NvBufferDestroy(m_compositedFrame);

    for (uint32_t i = 0; i < m_cameraModules.size(); i++) {
        if (m_dmabufs[i])
            NvBufferDestroy(m_dmabufs[i]);

		if (videoRecorder[i]) { 
	        videoRecorder[i]->CloseVideoRecordStream();
	    videoRecorder[i] = NULL;
	}
    }
}

bool ProcessFrameThread::threadInitialize()
{
    uint32_t tileCount = ceil(sqrt(m_cameraModules.size()));
    NvBufferCreateParams input_params = {0};
    NvBufferRect dstCompRect[m_cameraModules.size()];

    if (m_renderer)
    {
	int32_t cellWidth = m_windowSize.width() / tileCount;
	int32_t cellHeight = m_windowSize.height() / tileCount;

	for (uint32_t i = 0; i < m_cameraModules.size(); i++)
	{
	    dstCompRect[i].top  =  floor((i/2)) * cellHeight;
	    dstCompRect[i].left = i % 2 ? cellWidth : 0;
	    dstCompRect[i].width = cellWidth;
	    dstCompRect[i].height = cellHeight;

	    // Alignment fix
	    if (m_cameraModules.size() >= 5) {
	        dstCompRect[i].left += (cellWidth / 2);
	    }
	}
	// Allocate composited buffer
	input_params.payloadType = NvBufferPayload_SurfArray;
	input_params.width = m_windowSize.width();
	input_params.height = m_windowSize.height();
	input_params.layout = NvBufferLayout_Pitch;
	input_params.colorFormat = NvBufferColorFormat_NV12;
	input_params.nvbuf_tag = NvBufferTag_VIDEO_CONVERT;

	NvBufferCreateEx (&m_compositedFrame, &input_params);
	if (!m_compositedFrame)
	    ORIGINATE_ERROR("Failed to allocate composited buffer");

	// Initialize composite parameters
	memset(&m_compositeParam, 0, sizeof(m_compositeParam));
	m_compositeParam.composite_flag = NVBUFFER_COMPOSITE;
	m_compositeParam.input_buf_count = m_cameraModules.size();
	memcpy(m_compositeParam.dst_comp_rect, dstCompRect,
	    sizeof(NvBufferRect) * m_compositeParam.input_buf_count);

	for (uint32_t i = 0; i < m_cameraModules.size(); i++)
	{
	    IEGLOutputStream *iEglOutputStreams = interface_cast<IEGLOutputStream>(
							m_cameraModules[i]->getStream());
	    if (!iEglOutputStreams) 
		ORIGINATE_ERROR("Failed to get IEGLOutputStream Interface");

	    m_compositeParam.dst_comp_rect_alpha[i] = 1.0f;
	    m_compositeParam.src_comp_rect[i].top = 0;
	    m_compositeParam.src_comp_rect[i].left = 0;
	    m_compositeParam.src_comp_rect[i].width = iEglOutputStreams->getResolution().width();
	    m_compositeParam.src_comp_rect[i].height = iEglOutputStreams->getResolution().height();
	}

    }
    // Initialize buffer handles. Buffer will be created by FrameConsumer
    memset(m_dmabufs, 0, sizeof(m_dmabufs));

    // Create the FrameConsumer.
    for (uint32_t i = 0; i < m_cameraModules.size(); i++)
    {
        m_consumers[i].reset(FrameConsumer::create(m_cameraModules[i]->getStream()));
	videoRecorder[i] = NULL;
    }

    return true;
}

bool ProcessFrameThread::threadExecute()
{
    IEGLOutputStream *iEglOutputStreams[m_cameraModules.size()];
    IFrameConsumer *iFrameConsumers[m_cameraModules.size()];
    for (uint32_t i = 0; i < m_cameraModules.size(); i++)
    {
        iEglOutputStreams[i] = interface_cast<IEGLOutputStream>(m_cameraModules[i]->getStream());
        iFrameConsumers[i] = interface_cast<IFrameConsumer>(m_consumers[i]);
        if (!iFrameConsumers[i])
            ORIGINATE_ERROR("Failed to get IFrameConsumer interface");

        // Wait until the producer has connected to the stream.
        if (iEglOutputStreams[i]->waitUntilConnected() != STATUS_OK)
            ORIGINATE_ERROR("Stream failed to connect.");

	// Calibrating MasterDevice Before Capturing
	if (g_masterDev) {
		g_deviceID = 0;
		g_calibrate = true;
		PROPAGATE_ERROR(disableStreaming());
		PROPAGATE_ERROR(enableStreaming());
		g_calibrate = false;
	}
    }

    while (g_streamState)
    {
        std::vector<Image*> imageBuffers;
        UniqueObj<Frame> frameSet[m_cameraModules.size()];
	uint8_t skipFlag = 0;
	std::vector<Frame*> frmvector;

        for (uint32_t i = 0; i < m_cameraModules.size(); i++)
        {
            // Acquire a frame.
            frameSet[i] = UniqueObj<Frame>(iFrameConsumers[i]->acquireFrame());
            IFrame *iFrame = interface_cast<IFrame>(frameSet[i]);
            if (!iFrame)
                break;

 	    frmvector.push_back(frameSet[i].get());
	    imageBuffers.push_back(iFrame->getImage());

            // Get the IImageNativeBuffer extension interface.
            NV::IImageNativeBuffer *iNativeBuffer =
                interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
            if (!iNativeBuffer)
                ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");

            // If we don't already have a buffer, create one from this image.
            // Otherwise, just blit to our buffer.
            if (!m_dmabufs[i]) {
                m_dmabufs[i] = iNativeBuffer->createNvBuffer(iEglOutputStreams[i]->getResolution(),
                                                          NvBufferColorFormat_YUV420,
                                                          NvBufferLayout_BlockLinear);
                if (!m_dmabufs[i])
                    CONSUMER_PRINT("\tFailed to create NvBuffer\n");

            } else if (iNativeBuffer->copyToNvBuffer(m_dmabufs[i]) != STATUS_OK)
            	ORIGINATE_ERROR("Failed to copy frame to NvBuffer.");

	    if (!g_streamState) 
		break;

        }
	if (!g_streamState)
	    break;

	// Check the Sync of Sensor and get correct frames
	if (g_masterDev) {
	    Check_Sync(frmvector, &skipFlag); 

	    if (skipFlag & 0xAA)
	    	skipFlag &= 0xAA;
	    else if (skipFlag & 0x55)
		skipFlag = (~skipFlag) & 0x55;
	    else
		skipFlag = 0xff;

	    // Sync Logic Implementation. Dequeuing buffer if not in Sync
	    if (skipFlag != 0xff) {
		    bool SyncMissed = false;
		    for (uint32_t i = 0; i < m_cameraModules.size(); i++)
		    {
			if ((skipFlag >> i*2) & 0x03) 
			    continue;

			else {
			    frameSet[i].reset();
			    frameSet[i] = UniqueObj<Frame>(iFrameConsumers[i]->acquireFrame());
			    IFrame *iFrame = interface_cast<IFrame>(frameSet[i]);
			    if (!iFrame)
				    break;

			    frmvector.erase(frmvector.begin()+i);
			    frmvector.insert(frmvector.begin()+i, frameSet[i].get());
			    imageBuffers.erase(imageBuffers.begin()+i);
			    imageBuffers.insert(imageBuffers.begin()+i, iFrame->getImage());
			    SyncMissed = true;
			}
		    }
		    if (SyncMissed)
			    out_sync++;

		    if(!Check_Sync(frmvector, &skipFlag))
			    out_sync++;
	    }
	}
	
	total_frames++;

	// Calibrating Master Device for Sync
	if (out_sync % 10 == 0) {
	    if (g_masterDev) {
		g_deviceID = 0;
		g_calibrate = true;
		PROPAGATE_ERROR(disableStreaming());
		PROPAGATE_ERROR(enableStreaming());
		g_calibrate = false;
	    }
	    out_sync++;
	}

	// Control Flags
	if (g_captureState) {
	    PROPAGATE_ERROR(SaveImagefile(imageBuffers[g_deviceID]));
	    g_captureState = false;
	    g_captureType  = 0;
	}

	if (g_burstCaptureFlag) {
	    PROPAGATE_ERROR(BurstCapture(imageBuffers));
	    g_burstCaptureFlag = false;
	    g_captureType = 0;
	}

	if (g_getStreamMetadata) { 
	    PROPAGATE_ERROR(DisplayFrameMetadata(GetStreamMetadata(frmvector[g_deviceID])));
	    while(g_getStreamMetadata);
	}

	if (g_burstRecordFlag) {
	    g_calibrate = true;
	    PROPAGATE_ERROR(BurstRecordStart());
	    g_calibrate = false;
	    g_burstRecordFlag = false;
	}

	if (g_burstStopRecord) {
    	    g_calibrate = true;
	    PROPAGATE_ERROR(BurstRecordStop());
	    g_calibrate = false;
	    g_burstStopRecord = false;
	}

	if (g_videoRecordFlag) {
	    g_calibrate = true;
	    PROPAGATE_ERROR(VideoRecordStart());
	    g_calibrate = false;
	    g_videoRecordFlag = false;
	}

	if (g_videoStopRecord) {
	    g_calibrate = true;
	    PROPAGATE_ERROR(VideoRecordStop());
	    g_calibrate = false;
	    g_videoStopRecord = false;
	}

	// Display Renderer
	if (m_renderer) {
	    if (m_cameraModules.size() > 1)
	    {
		// Composite multiple input to one frame
		NvBufferComposite(m_dmabufs, m_compositedFrame, &m_compositeParam);
		m_renderer->render(m_compositedFrame);
	    }
	    else
		m_renderer->render(m_dmabufs[0]);
	}

	// Clearing Buffers for next Frame Set
	imageBuffers.clear();
	frmvector.clear();
	for (uint32_t i=0; i < m_cameraModules.size(); i++)
	    frameSet[i].reset();
    }
    requestShutdown();

    CONSUMER_PRINT("Done.\n");

    return true;
}

bool ProcessFrameThread::threadShutdown()
{
    // Closing Video Threads If any present
    for (uint32_t i=0; i < m_cameraModules.size(); i++) {
	if (videoRecorder[i] != NULL) {
	    g_deviceID = i;
	    g_videoStopRecord = true;
	    PROPAGATE_ERROR(disableStreaming());
	    PROPAGATE_ERROR(videoRecorder[i]->CloseVideoRecordStream());
	    PROPAGATE_ERROR(enableStreaming());
	    videoRecorder[i] = NULL;
	}
    }

    std::cout << "Total No. Frame. Sets : " << total_frames << std::endl;

    if (g_masterDev) {
        std::cout << "Total out of Sync : " << out_sync << std::endl;
        std::cout << "Total in Sync : " << total_frames - out_sync << std::endl;
    }
    return true;
}

bool ProcessFrameThread::enableStreaming()
{
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_cameraModules[g_deviceID]->getSession());
    if (!iCaptureSession)
        ORIGINATE_ERROR("Failed to get ICaptureSession interface");

    // Submit capture requests.
    if (iCaptureSession->repeat(m_cameraModules[g_deviceID]->getRequest()) != STATUS_OK)
	ORIGINATE_ERROR("Failed to start repeat capture request");

    IEGLOutputStream *iStream = interface_cast<IEGLOutputStream>(m_cameraModules[g_deviceID]->getStream());
    if (!iStream)
	ORIGINATE_ERROR("Failed to get IEGLOutputStream interface");

    if (iStream->waitUntilConnected() != STATUS_OK)
        ORIGINATE_ERROR("Stream failed to connect.");

    return true;
}

bool ProcessFrameThread::disableStreaming()
{
    // Stop the repeating request and wait for idle.
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_cameraModules[g_deviceID]->getSession());
    if (!iCaptureSession)
	ORIGINATE_ERROR("Failed to get ICaptureSession interface");
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();

    return true;
}

bool ProcessFrameThread::SaveImagefile(Image *image)
{
    std::ostringstream fileName;
    struct tm 	 *tstruct;
    char 	  time_buf[80], *homedir = getenv("HOME");
    time_t 	  cur_time = time(0);
    Argus::Status status;
    NvBufferParams params;
    // Create File Name and write Image 
    tstruct	= localtime(&cur_time);

    sprintf(time_buf, "%d-%d-%d_%dh.%dm.%ds", tstruct->tm_mday, tstruct->tm_mon+1, tstruct->tm_year+1900, tstruct->tm_hour,
						tstruct->tm_min, tstruct->tm_sec);
    fileName << homedir << "/IMG_cam_" << g_deviceID << "_p_" << getpid() << "_" << time_buf;

    if (g_captureType == CAPTURE_TYPE_JPEG) {
	fileName << ".jpg";
	EGLStream::IImageJPEG *iImageJPEG = interface_cast<EGLStream::IImageJPEG>(image);
	if (!iImageJPEG)
	    ORIGINATE_ERROR("Failed to create IJpeg Interface");

	if (iImageJPEG->writeJPEG(fileName.str().c_str()) != STATUS_OK)
	    ORIGINATE_ERROR("Failed to write JPEG");

    } else if (g_captureType == CAPTURE_TYPE_RAW) {
	fileName << ".nv12";
	ISensorMode *iSensorMode = interface_cast<ISensorMode>(m_cameraModules[g_deviceID]->getSensorMode());
	if (!iSensorMode)
	    ORIGINATE_ERROR("Failed to get Sensor Mode interface");

	EGLStream::NV::IImageNativeBuffer *iImageNativeBuffer =
			interface_cast<EGLStream::NV::IImageNativeBuffer>(image);
	if (!iImageNativeBuffer)
	    ORIGINATE_ERROR("Failed to get IImageNative Buffer Interface");

	int fd = iImageNativeBuffer->createNvBuffer(iSensorMode->getResolution(), NvBufferColorFormat_YUV420, NvBufferLayout_Pitch, NV::ROTATION_0, &status);
	if (status != STATUS_OK)
	    ORIGINATE_ERROR("Failed to create Native Buffer");

	NvBufferGetParams(fd, &params);
	FILE *file = fopen(fileName.str().c_str(), "wb");
	if (!file)
	    ORIGINATE_ERROR("No Memory to create File");

	for (uint32_t i = 0; i < params.num_planes; i++) {
	    size_t size = params.height[i] * params.pitch[i];

	    char *vptr = (char*)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, params.offset[i]);
	    for (uint32_t j = 0; j < params.height[i]; j++) { 
                    fwrite(vptr+params.pitch[i]*j, params.width[i], 1, file);
            }
	    munmap(vptr, size);
	}
	fclose(file);
    } else {
	/* Need to implement Bayer Capture */
    }

    printf("\n");
    drawLine();
    std::cout << tabSpace() << "Image Saved to\t" << tabSpace() << fileName.str().c_str() << std::endl;
    drawLine();

    return true; 
}

bool ProcessFrameThread::BurstRecordStart()
{
    for (uint32_t i = 0 ; i < m_cameraModules.size(); i++) {
   	g_deviceID = i;
   	videoRecorder[i] = new VideoRecord(m_cameraModules[i]);
    	PROPAGATE_ERROR(disableStreaming());
	videoRecorder[i]->CreateVideoRecordStream();
	PROPAGATE_ERROR(enableStreaming());
    }

    // Calibrating Master Device for Sync
    if (g_masterDev) {
	g_deviceID = 0;
	PROPAGATE_ERROR(disableStreaming());
	PROPAGATE_ERROR(enableStreaming());
    }
    return true;
}

bool ProcessFrameThread::VideoRecordStart()
{
    videoRecorder[g_deviceID] = new VideoRecord(m_cameraModules[g_deviceID]);
    PROPAGATE_ERROR(disableStreaming());
    videoRecorder[g_deviceID]->CreateVideoRecordStream();
    PROPAGATE_ERROR(enableStreaming());

    // Calibrating Master Device for Sync
    if (g_masterDev) {
	tmp_devID = g_deviceID;
	g_deviceID = 0;
	PROPAGATE_ERROR(disableStreaming());
	PROPAGATE_ERROR(enableStreaming());
        g_deviceID = tmp_devID;
    }
    return true;
}

bool ProcessFrameThread::VideoRecordStop()
{
    PROPAGATE_ERROR(disableStreaming());
    PROPAGATE_ERROR(videoRecorder[g_deviceID]->CloseVideoRecordStream());
    PROPAGATE_ERROR(enableStreaming());
    videoRecorder[g_deviceID] = NULL;

    // Calibrating Master Device for Sync
    if (g_masterDev) {
	g_deviceID = 0;
	PROPAGATE_ERROR(disableStreaming());
	PROPAGATE_ERROR(enableStreaming());
    }
    return true;
}

bool ProcessFrameThread::BurstRecordStop()
{
    for (uint32_t i = 0 ; i < m_cameraModules.size(); i++) {
	g_deviceID = i;
	g_videoStopRecord = true;
    	PROPAGATE_ERROR(disableStreaming());
	PROPAGATE_ERROR(videoRecorder[i]->CloseVideoRecordStream());
    	PROPAGATE_ERROR(enableStreaming());
    	videoRecorder[i] = NULL;
	g_videoStopRecord = false;
    }

    // Calibrating Master Device for Sync
    if (g_masterDev) {
	g_deviceID = 0;
	PROPAGATE_ERROR(disableStreaming());
	PROPAGATE_ERROR(enableStreaming());
    }
    return true;
}
bool ProcessFrameThread::BurstCapture(std::vector<Image*> images)
{
    for(uint32_t i=0; i < images.size(); i++) {
	g_deviceID = i;
	PROPAGATE_ERROR(SaveImagefile(images[i]));
    }
    return true;
}

CaptureMetadata* ProcessFrameThread::GetStreamMetadata(Frame *frame)
{
    IArgusCaptureMetadata *iArgusCaptureMetadata = interface_cast<IArgusCaptureMetadata>(frame);
    if (!iArgusCaptureMetadata)
	return NULL;

    return iArgusCaptureMetadata->getMetadata();
}

bool ProcessFrameThread::DisplayFrameMetadata(CaptureMetadata *Metadata)
{
    ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(Metadata);
    if (!iMetadata)
	ORIGINATE_ERROR("Failed to get Metadata interface");

    printf("\n");
    drawHeader();
    std::cout << tabSpace() << "\t Camera Device \t\t" << tabSpace() << g_deviceID << std::endl;
    drawHeader();
    std::cout << tabSpace() << "\t Capture API ID\t\t" << tabSpace() << iMetadata->getCaptureId() << std::endl;  
    drawLine();
    std::cout << tabSpace() << "\t AE Lock Stat\t\t" << tabSpace() << iMetadata->getAeLocked() << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Frame Duration(ms)\t" << tabSpace() << 
					iMetadata->getFrameDuration()/1000000 << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Stream FrameRate\t" << tabSpace() << 
					(1000000000/iMetadata->getFrameDuration()) << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t ISP Dig. Gain\t\t" << tabSpace() << iMetadata->getIspDigitalGain() << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Frame Readout\t\t" << tabSpace() << iMetadata->getFrameReadoutTime()/1000 << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Frame Scene Lux\t" << tabSpace() << iMetadata->getSceneLux() << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Sensor Gain\t\t" << tabSpace() << iMetadata->getSensorAnalogGain() << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Sensor Exposure\t" << tabSpace() << iMetadata->getSensorExposureTime()/1000 << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Sensor ISO Sens.\t" << tabSpace() << iMetadata->getSensorSensitivity() << std::endl;
    drawLine();
    std::cout << tabSpace() << "\t Sensor TimeStamp\t" << tabSpace() << iMetadata->getSensorTimestamp()/1000 << std::endl;
    drawLine();

    g_getStreamMetadata = false;
    return true;
}

bool ProcessFrameThread::Check_Sync(std::vector<Frame*> frames, uint8_t *skip)
{
    int32_t Diff_ts = 0, Diff_ms;
    bool SyncMode = true;

    for (uint32_t i=0 ; i < frames.size(); i++) {
	CaptureMetadata* Metadata = GetStreamMetadata(frames[i]);
	ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(Metadata);

	if (i == 0)
	    Diff_ts = iMetadata->getSensorTimestamp()/1000000;

	else {
	    Diff_ms = (iMetadata->getSensorTimestamp()/1000000) - Diff_ts;
	    if ((Diff_ms < -4 || Diff_ms > 4) && SyncMode) 
		SyncMode = false;

	    if (Diff_ms > 4)
		*skip |= (2 << i*2);

	    else if (Diff_ms < -4)
		*skip |= (1 << i*2);
	}
    }
    return SyncMode;
}

}; //NameSpace ArgusSamples
