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

using namespace Argus;
using namespace EGLStream;

namespace ArgusSamples
{

bool VideoRecord::CreateVideoRecordStream()
{
    // Output FileName
    struct tm 	 *tstruct;
    char 	  time_buf[80], *homedir = getenv("HOME");
    time_t 	  cur_time = time(0);
    std::ostringstream outputFileName, pipeline_name;

    ISensorMode *iSensorMode = interface_cast<ISensorMode>(m_cameraModule->getSensorMode());
    if (!iSensorMode)
	ORIGINATE_ERROR("Failed to get ISensorMode Interface");

    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(m_cameraModule->getSession());
    if (!iCaptureSession)
        ORIGINATE_ERROR("Failed to get ICaptureSession interface");

    UniqueObj<OutputStreamSettings> streamSettings(
			iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    IEGLOutputStreamSettings *iEglOutputStreamSettings = 
			interface_cast<IEGLOutputStreamSettings>(streamSettings);
    if (!iEglOutputStreamSettings)
	ORIGINATE_ERROR("Failed to get iEGL Ouput Stream settings");

    iEglOutputStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEglOutputStreamSettings->setResolution(iSensorMode->getResolution());
    iEglOutputStreamSettings->setMetadataEnable(true);

    m_outputStream.reset(iCaptureSession->createOutputStream(streamSettings.get()));
    
    IRequest *iRequest = interface_cast<IRequest>(m_cameraModule->getRequest());
    if (!iRequest)
	ORIGINATE_ERROR("Failed to create Request");

    iRequest->enableOutputStream(m_outputStream.get());

    if (g_captureType == CAPTURE_TYPE_H264) {
        gst_init(0, NULL);
        main_loop = g_main_loop_new(NULL,FALSE);
        char launch_string_[1024];

        // Create File Name and write Image 
        tstruct	= localtime(&cur_time);
        sprintf(time_buf, "%dh-%dm-%ds", tstruct->tm_hour, tstruct->tm_min, tstruct->tm_sec);

        outputFileName << homedir << "/VID_cam_" << g_deviceID << "_p_" << getpid() << "_" << time_buf; 

    	printf("\n");
    	drawLine();
    	std::cout << tabSpace() << "Saving Video to\t" << tabSpace() << outputFileName.str().c_str() << ".mp4" << std::endl;
    	drawLine();

    	pipeline_name << "source_dev-" << g_deviceID;

    	sprintf(launch_string_,
		"appsrc name=%s ! video/x-h264,width=%d,height=%d,stream-format=byte-stream !", pipeline_name.str().c_str(), 
				iSensorMode->getResolution().width(), iSensorMode->getResolution().height());

    	sprintf(launch_string_ + strlen(launch_string_),
			" h264parse ! qtmux ! filesink location=%s.mp4", outputFileName.str().c_str());
    	gst_pipeline = (GstPipeline*)gst_parse_launch(launch_string_, NULL);
    	appsrc_ = gst_bin_get_by_name(GST_BIN(gst_pipeline), pipeline_name.str().c_str());
    	gst_app_src_set_stream_type(GST_APP_SRC(appsrc_), GST_APP_STREAM_TYPE_STREAM);
    	gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_PLAYING);

    	m_frameConsumer = new VideoEncoderThread(m_outputStream.get(), appsrc_, m_cameraModule->getSensorMode());
    	PROPAGATE_ERROR(m_frameConsumer->initialize());
    	PROPAGATE_ERROR(m_frameConsumer->waitRunning());
    } else {
	m_rawFrameConsumer = new RawVideoRecorder(m_outputStream.get(), m_cameraModule->getSensorMode());
	PROPAGATE_ERROR(m_rawFrameConsumer->initialize());
	PROPAGATE_ERROR(m_rawFrameConsumer->waitRunning());
    }

    return true;
}

bool VideoRecord::CloseVideoRecordStream()
{

    // Wait for the consumer thread to complete.
    if (g_captureType == CAPTURE_TYPE_H264) {
    	PROPAGATE_ERROR(m_frameConsumer->shutdown());
    	m_frameConsumer = NULL;

    	gst_element_set_state((GstElement*)gst_pipeline, GST_STATE_NULL);
    	gst_object_unref(GST_OBJECT(gst_pipeline));
    	g_main_loop_unref(main_loop);

    } else {
	PROPAGATE_ERROR(m_rawFrameConsumer->shutdown());
	m_rawFrameConsumer = NULL;
    }

    IEGLOutputStream *iStream = interface_cast<IEGLOutputStream>(m_outputStream.get());
    if (!iStream)
	ORIGINATE_ERROR("Failed to get IEGLOutputStream interface");

    iStream->disconnect();

    IRequest *iRequest = interface_cast<IRequest>(m_cameraModule->getRequest());
    if (!iRequest)
	ORIGINATE_ERROR("Failed to create Request");

    iRequest->disableOutputStream(m_outputStream.get());

    // Destroy the output stream to end the consumer thread.
    if (m_outputStream)
	m_outputStream.reset();

    PRODUCER_PRINT("Done -- exiting.\n");
    return true;
}

RawVideoRecorder::RawVideoRecorder(OutputStream *stream, SensorMode *sensorMode)
    : m_stream(stream)
    , m_initialized(false)
    , m_initOnce(false)
    , m_sensorMode(sensorMode)
{
}

RawVideoRecorder::~RawVideoRecorder()
{
    if(fileptr)
	fclose(fileptr);
}


bool RawVideoRecorder::threadInitialize()
{
    if (m_initialized)
	return true;

    // Output FileName
    struct tm 	 *tstruct;
    char 	  time_buf[80], *homedir = getenv("HOME");
    time_t 	  cur_time = time(0);
    tstruct	= localtime(&cur_time);
    sprintf(time_buf,"%d-%d-%d", tstruct->tm_hour, tstruct->tm_min, tstruct->tm_sec);

    m_fileName << homedir << "/VID_cam_" << g_deviceID << "_p_" << getpid() << "_" << time_buf << ".raw";

    printf("\n");
    drawLine();
    std::cout << tabSpace() << "Saving uncompressed RAW video to\t" << tabSpace() << m_fileName.str().c_str() << std::endl;
    drawLine();

    // Create the FrameConsumer.
    m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
    if (!m_consumer)
        ORIGINATE_ERROR("Failed to create FrameConsumer");

    fileptr = (FILE*) fopen(m_fileName.str().c_str(), "wb");
    if (fileptr == NULL) {
	printf("Error - %s\n", strerror(errno));
	ORIGINATE_ERROR("No Memory to create File");
    }

    m_initialized = true;
    return true;
}

bool RawVideoRecorder::threadExecute()
{
    IEGLOutputStream *iStream = interface_cast<IEGLOutputStream>(m_stream);
    IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer);
    Argus::Status status;
    UniqueObj<Frame> m_frame;

    if (!m_initOnce) {

    	// Wait until the producer has connected to the stream.
    	if (iStream->waitUntilConnected() != STATUS_OK)
    	    ORIGINATE_ERROR("Stream failed to connect.");

	m_frame = UniqueObj<Frame>(iFrameConsumer->acquireFrame());
	IFrame *iFrame = interface_cast<IFrame>(m_frame);
	if (!iFrame)
	    ORIGINATE_ERROR("Failed to get iFrame interface");

	EGLStream::Image *image = iFrame->getImage();
	if (!image)
	    ORIGINATE_ERROR("Failed to get image.");

	EGLStream::NV::IImageNativeBuffer *iImageNativeBuffer = 
			interface_cast<EGLStream::NV::IImageNativeBuffer>(image);
	if (!iImageNativeBuffer)
	    ORIGINATE_ERROR("Failed to get iImageNativeBuffer interface");

	m_fd = iImageNativeBuffer->createNvBuffer(iStream->getResolution(), NvBufferColorFormat_YUV420, NvBufferLayout_Pitch, EGLStream::NV::ROTATION_0, &status);
	if (status != STATUS_OK)
	    ORIGINATE_ERROR("Failed to create NvBuffer");

	NvBufferGetParams(m_fd, &m_params);
        for (uint32_t i = 0; i < m_params.num_planes; i++)
	    Buff_size[i] = m_params.height[i] * m_params.pitch[i];

	m_frame.reset();	
	m_initOnce = true;
    }
    m_frame = UniqueObj<Frame>(iFrameConsumer->acquireFrame());
    IFrame *iFrame = interface_cast<IFrame>(m_frame);
    if (!iFrame)
    	ORIGINATE_ERROR("Failed to get iFrame interface");

    EGLStream::Image *image = iFrame->getImage();
    if (!image)
	ORIGINATE_ERROR("Failed to get image.");

    EGLStream::NV::IImageNativeBuffer *iImageNativeBuffer = 
			interface_cast<EGLStream::NV::IImageNativeBuffer>(image);
    if (!iImageNativeBuffer)
        ORIGINATE_ERROR("Failed to get iImageNativeBuffer interface");

    if (iImageNativeBuffer->copyToNvBuffer(m_fd, EGLStream::NV::ROTATION_0) != STATUS_OK)
    	ORIGINATE_ERROR("Failed to create NvBuffer");

    for (uint32_t i = 0; i < m_params.num_planes; i++) {
	char *vptr = (char*)mmap(NULL, Buff_size[i], PROT_READ | PROT_WRITE, MAP_SHARED, m_fd, m_params.offset[i]);

	for (uint32_t j = 0; j < m_params.height[i]; j++) 
            fwrite(vptr+m_params.pitch[i]*j, m_params.width[i], 1, fileptr);

	munmap(vptr, Buff_size[i]);
    }
    return true;
}

bool RawVideoRecorder::threadShutdown()
{
    if (!m_initialized)
	return true;

    if (fileptr)
	fclose(fileptr);

    m_initialized = false;
    m_initOnce = false;
    return true;	
}

// H264 Video Recording
VideoEncoderThread::VideoEncoderThread(OutputStream* stream,GstElement *appsrc_, SensorMode *sensorMode) :
        m_stream(stream),
        m_VideoEncoder(NULL),
        m_gotError(false),
	m_appsrc_(appsrc_),
	m_sensorMode(sensorMode),
	timestamp(0),
	timestamp_diff(0)
{
}

VideoEncoderThread::~VideoEncoderThread()
{
    if (m_VideoEncoder)
    {
        delete m_VideoEncoder;
    }
    gst_deinit();
}

bool VideoEncoderThread::threadInitialize()
{

    // Create the FrameConsumer.
    m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
    if (!m_consumer)
        ORIGINATE_ERROR("Failed to create FrameConsumer");

    // Create Video Encoder
    if (!createVideoEncoder())
        ORIGINATE_ERROR("Failed to create video m_VideoEncoderoder");

    CONSUMER_PRINT("Created Video Encoder Pipeline\n");

    // Stream on
    int e = m_VideoEncoder->output_plane.setStreamStatus(true);
    if (e < 0)
        ORIGINATE_ERROR("Failed to stream on output plane");
    e = m_VideoEncoder->capture_plane.setStreamStatus(true);
    if (e < 0)
        ORIGINATE_ERROR("Failed to stream on capture plane");

    // Set video encoder callback
    m_VideoEncoder->capture_plane.setDQThreadCallback(encoderCapturePlaneDqCallback);

    // startDQThread starts a thread internally which calls the
    // encoderCapturePlaneDqCallback whenever a buffer is dequeued
    // on the plane
    m_VideoEncoder->capture_plane.startDQThread(this);

    // Enqueue all the empty capture plane buffers
    for (uint32_t i = 0; i < m_VideoEncoder->capture_plane.getNumBuffers(); i++)
    {
        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.index = i;
        v4l2_buf.m.planes = planes;

        CHECK_ERROR(m_VideoEncoder->capture_plane.qBuffer(v4l2_buf, NULL));
    }

    return true;
}

bool VideoEncoderThread::threadExecute()
{
    IEGLOutputStream *iStream = interface_cast<IEGLOutputStream>(m_stream);
    IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer);

    // Wait until the producer has connected to the stream.
    if (iStream->waitUntilConnected() != STATUS_OK)
        ORIGINATE_ERROR("Stream failed to connect.");

    int bufferIndex;

    bufferIndex = 0;

    // Keep acquire frames and queue into encoder
    while (!m_gotError)
    {
        NvBuffer *buffer;
        int fd = -1;

        struct v4l2_buffer v4l2_buf;
        struct v4l2_plane planes[MAX_PLANES];

        memset(&v4l2_buf, 0, sizeof(v4l2_buf));
        memset(planes, 0, MAX_PLANES * sizeof(struct v4l2_plane));

        v4l2_buf.m.planes = planes;

        // Check if we need dqBuffer first
        if (bufferIndex < MAX_ENCODER_FRAMES &&
            m_VideoEncoder->output_plane.getNumQueuedBuffers() <
            m_VideoEncoder->output_plane.getNumBuffers())
        {
            // The queue is not full, no need to dqBuffer
            // Prepare buffer index for the following qBuffer
            v4l2_buf.index = bufferIndex++;
        }
        else
        {
            // Output plane full or max outstanding number reached
            CHECK_ERROR(m_VideoEncoder->output_plane.dqBuffer(v4l2_buf, &buffer,
                                                              NULL, 10));
            // Release the frame.
            fd = v4l2_buf.m.planes[0].m.fd;
            NvBufferDestroy(fd);
        }

	if (g_videoStopRecord || !g_streamState) {
            // Send EOS
	    v4l2_buf.m.planes[0].m.fd = fd;
            v4l2_buf.m.planes[0].bytesused = 0;
            CHECK_ERROR(m_VideoEncoder->output_plane.qBuffer(v4l2_buf, NULL));
            break;
	}

        // Acquire a frame.
        UniqueObj<Frame> frame(iFrameConsumer->acquireFrame());
        IFrame *iFrame = interface_cast<IFrame>(frame);
        if (!iFrame)
        {
            // Send EOS
            v4l2_buf.m.planes[0].m.fd = fd;
            v4l2_buf.m.planes[0].bytesused = 0;
            CHECK_ERROR(m_VideoEncoder->output_plane.qBuffer(v4l2_buf, NULL));
            break;
        }

	IArgusCaptureMetadata *iArgusCaptureMetdata = interface_cast<IArgusCaptureMetadata>(frame);
	if (!iArgusCaptureMetdata)
		ORIGINATE_ERROR("Failed to get IArgus Capture Metadata");

	m_metadata = iArgusCaptureMetdata->getMetadata();
    	ICaptureMetadata *iMetadata = interface_cast<ICaptureMetadata>(m_metadata);
    	if (!iMetadata)
	    ORIGINATE_ERROR("Failed to get ICapture Metadata");

	timestamp_diff = iMetadata->getFrameDuration(); //ns
	
        // Get the IImageNativeBuffer extension interface and create the fd.
        NV::IImageNativeBuffer *iNativeBuffer =
            interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
        if (!iNativeBuffer)
            ORIGINATE_ERROR("IImageNativeBuffer not supported by Image.");
        fd = iNativeBuffer->createNvBuffer(iStream->getResolution(),
                                           NvBufferColorFormat_YUV420,
                                           NvBufferLayout_BlockLinear);

        // Push the frame into V4L2.
        v4l2_buf.m.planes[0].m.fd = fd;
        v4l2_buf.m.planes[0].bytesused = 1; // byteused must be non-zero
        CHECK_ERROR(m_VideoEncoder->output_plane.qBuffer(v4l2_buf, NULL));
    }

    // Wait till capture plane DQ Thread finishes
    // i.e. all the capture plane buffers are dequeued
    m_VideoEncoder->capture_plane.waitForDQThread(2000);

    CONSUMER_PRINT("Closed Recording Pipeline.\n");

    requestShutdown();

    return true;
}

bool VideoEncoderThread::threadShutdown()
{
    return true;
}

bool VideoEncoderThread::createVideoEncoder()
{
    int ret = 0;
    std::ostringstream encoderName;
    IEGLOutputStream *iEglOutputStream = interface_cast<IEGLOutputStream>(m_stream);
    float framerate = DEFAULT_FPS;

    // Create ISensorMode interface for updating Plane buffer params
    ISensorMode *iSensorMode = interface_cast<ISensorMode>(m_sensorMode);
    if (!iSensorMode)
	ORIGINATE_ERROR("Failed to get ISensor Mode interface");

    framerate = TimeValue::fromNSec(iSensorMode->getFrameDurationRange().min()).toCyclesPerSec();
 
    encoderName << "encoder-dev_" << g_deviceID; 
    m_VideoEncoder = NvVideoEncoder::createVideoEncoder(encoderName.str().c_str());
    if (!m_VideoEncoder)
        ORIGINATE_ERROR("Could not create m_VideoEncoderoder");

    ret = m_VideoEncoder->setCapturePlaneFormat(V4L2_PIX_FMT_H264, iEglOutputStream->getResolution().width(),
                                    iEglOutputStream->getResolution().height(), 2 * iEglOutputStream->getResolution().area() );
    if (ret < 0)
        ORIGINATE_ERROR("Could not set capture plane format");

    ret = m_VideoEncoder->setOutputPlaneFormat(V4L2_PIX_FMT_YUV420M, iEglOutputStream->getResolution().width(),
                                    iEglOutputStream->getResolution().height());
    if (ret < 0)
        ORIGINATE_ERROR("Could not set output plane format");

    ret = m_VideoEncoder->setBitrate(4 * iEglOutputStream->getResolution().area());
    if (ret < 0)
        ORIGINATE_ERROR("Could not set bitrate");

    ret = m_VideoEncoder->setProfile(V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED);

    if (ret < 0)
        ORIGINATE_ERROR("Could not set m_VideoEncoderoder profile");

    ret = m_VideoEncoder->setLevel(V4L2_MPEG_VIDEO_H264_LEVEL_5_1);
    if (ret < 0)
        ORIGINATE_ERROR("Could not set m_VideoEncoderoder level");

    ret = m_VideoEncoder->setRateControlMode(V4L2_MPEG_VIDEO_BITRATE_MODE_CBR);
    if (ret < 0)
        ORIGINATE_ERROR("Could not set rate control mode");

    ret = m_VideoEncoder->setIFrameInterval(framerate);
    if (ret < 0)
        ORIGINATE_ERROR("Could not set I-frame interval");

    ret = m_VideoEncoder->setFrameRate(framerate, 1);
    if (ret < 0)
        ORIGINATE_ERROR("Could not set m_VideoEncoderoder framerate");

    ret = m_VideoEncoder->setHWPresetType(V4L2_ENC_HW_PRESET_SLOW);
    if (ret < 0)
        ORIGINATE_ERROR("Could not set m_VideoEncoderoder HW Preset");

    // Query, Export and Map the output plane buffers so that we can read
    // raw data into the buffers
    ret = m_VideoEncoder->output_plane.setupPlane(V4L2_MEMORY_DMABUF, 3, true, false);
    if (ret < 0)
        ORIGINATE_ERROR("Could not setup output plane");

    // Query, Export and Map the output plane buffers so that we can write
    // m_VideoEncoderoded data from the buffers
    ret = m_VideoEncoder->capture_plane.setupPlane(V4L2_MEMORY_MMAP, 3, true, false);
    if (ret < 0)
        ORIGINATE_ERROR("Could not setup capture plane");

    printf("create video encoder return true\n");
    return true;
}

void VideoEncoderThread::abort()
{
    m_VideoEncoder->abort();
    m_gotError = true;
}

bool VideoEncoderThread::encoderCapturePlaneDqCallback(struct v4l2_buffer *v4l2_buf,
                                                   NvBuffer * buffer,
                                                   NvBuffer * shared_buffer,
                                                   void *arg)
{
    VideoEncoderThread *thiz = (VideoEncoderThread*)arg;

    if (!v4l2_buf)
    {
        thiz->abort();
        ORIGINATE_ERROR("Failed to dequeue buffer from encoder capture plane");
    }

    if (buffer->planes[0].bytesused > 0) {
	GstBuffer  *gstbuf;
	GstMapInfo map = {0};
	GstFlowReturn ret;
	gstbuf = gst_buffer_new_allocate (NULL, buffer->planes[0].bytesused, NULL);
	gstbuf->pts = thiz->timestamp;
	thiz->timestamp += thiz->timestamp_diff;

	gst_buffer_map (gstbuf, &map, GST_MAP_WRITE);
	memcpy(map.data,buffer->planes[0].data, buffer->planes[0].bytesused);
	gst_buffer_unmap(gstbuf, &map);

	g_signal_emit_by_name( thiz->m_appsrc_, "push-buffer", gstbuf, &ret);
	gst_buffer_unref(gstbuf);
    } else {
	gst_app_src_end_of_stream((GstAppSrc *) thiz->m_appsrc_);
	sleep(1);
    }
    if (thiz->m_VideoEncoder->capture_plane.qBuffer(*v4l2_buf, NULL) < 0)
    {
        thiz->abort();
        ORIGINATE_ERROR("Failed to enqueue buffer to encoder capture plane");
        return false;
    }

    // GOT EOS from m_VideoEncoderoder. Stop dqthread.
    if (buffer->planes[0].bytesused == 0)
    {
        CONSUMER_PRINT("Got EOS, exiting...\n");
        return false;
    }

    return true;
}

}; // namespace ArgusSamples
