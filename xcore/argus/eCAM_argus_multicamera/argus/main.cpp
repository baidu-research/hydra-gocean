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

#include "main.h"

// Control Flags
bool g_getStreamMetadata = false;
bool g_burstCaptureFlag = false;
bool g_videoRecordFlag = false;
bool g_videoStopRecord = false;
bool g_burstRecordFlag = false;
bool g_burstStopRecord = false;
bool g_frameRateChange = false;
bool g_captureState = false;
bool g_calibrate = false;

float g_masterFrameRate = DEFAULT_FPS;

uint32_t g_deviceID = 0;
uint32_t g_masterDev = 0;
uint32_t g_streamState = 1;
uint32_t g_captureType = 0;
uint32_t g_controlState = 0;


using namespace Argus;
using namespace EGLStream;

void signal_handler( int signal_num ) { 
   std::cout << "\nCtrl + C pressed! (" << signal_num << ").Stopping Camera stream\n"; 
   std::cout << " Manual termination from User Interface thread required! \n" << std::endl;
   g_streamState = 0;
   g_calibrate = false;
   return;
}

namespace ArgusSamples
{

// Debug print macros.
#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)

UniqueObj<CameraProvider>  g_cameraProvider;
std::vector<CameraDevice*> g_cameraDevices;

/*******************************************************************************
 * Extended options class to add additional options specific to this code.
 ******************************************************************************/
class MultiCameraAppOptions : public CommonOptions
{
public:
    MultiCameraAppOptions(const char *programName)
        : CommonOptions(programName,
                        ArgusSamples::CommonOptions::Option_N_NumDevices |
                        ArgusSamples::CommonOptions::Option_P_Preview |
                        ArgusSamples::CommonOptions::Option_M_SensorMode )
	, m_masterMode(false)
    {
	addOption(createValueOption
		("MasterSlaveMode", 'm', "VALUE", "Master Slave Mode enable/disable", m_masterMode));
    }
    bool getMasterMode() const { return m_masterMode.get(); }
protected:
    Value<bool> m_masterMode;
};

static bool execute(const MultiCameraAppOptions& options)
{
    uint32_t displayWidth = 0, displayHeight = 0;
    std::vector<CameraModulesEGL*> 	cameraModules;					// Vector to handle all the camera modules
    UniqueObj<CameraModulesEGL> 	captureHolders[MAX_CAMERA_NUM];			// Camera Module wrapper class
    UniqueObj<CaptureSession>		cameraSessions[MAX_CAMERA_NUM];			// Capture Sessions for connected cameras
    std::vector<SensorMode*> 		sensormodes;					// Vector to store all the available sensor modes
    MasterSlaveMode			masterSlaveHandler(options.getMasterMode());	// Wrapper class to link v4l2 and master-slave control
    NvEglRenderer 			*m_renderer = NULL;				// Display renderer

    mainMenu("e-CAM Argus MultiCamera Command line Application");

    // Initialize EGL renderer if display is enabled.
    if (options.previewMode()) {
	NvEglRenderer::getDisplayResolution(displayWidth,displayHeight);
	m_renderer = NvEglRenderer::createEglRenderer("Renderer", displayWidth, displayHeight, 0, 0);
	if (!m_renderer)
	    ORIGINATE_ERROR("Failed to create EGLRenderer.");
    }

    // Initialize the Argus camera provider.
    g_cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());

    // Get the ICameraProvider interface from the global CameraProvider.
    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(g_cameraProvider);
    if (!iCameraProvider)
        ORIGINATE_ERROR("Failed to get ICameraProvider interface");

    // Get the camera devices.
    iCameraProvider->getCameraDevices(&g_cameraDevices);
    if (g_cameraDevices.size() == 0)
    {
        ORIGINATE_ERROR("there are %d cameras",(unsigned)g_cameraDevices.size());
    }
    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(g_cameraDevices[0]);
    if (!iCameraProperties)
	ORIGINATE_ERROR("Failed to get ICameraProperties Interface");

    // Get available Sensor Modes
    iCameraProperties->getAllSensorModes(&sensormodes);
    if (sensormodes.size() <= options.sensorModeIndex())
    {
        ORIGINATE_ERROR("Preview Sensor Mode %d not available; the sensor mode range is [0 - %d]",
                        options.sensorModeIndex(), (unsigned)sensormodes.size()-1);
    }

    ISensorMode *iSensorMode = interface_cast<ISensorMode>(sensormodes[options.sensorModeIndex()]);
    if (!iSensorMode)
	ORIGINATE_ERROR("Failed to get SensorMode interface");

    // Check if sensor mode provided is DOL WDR
    Ext::IDolWdrSensorMode *dolMode = interface_cast<Ext::IDolWdrSensorMode>(sensormodes[options.sensorModeIndex()]);

    // Set Master/Slave Mode according to UserInput
    if (options.getMasterMode()) {
	if ((options.numCameraDevices() <= MASTER_DEVNODE) || (g_cameraDevices.size() < MASTER_DEVNODE))
    	    std::cout << tabSpace() << YELLOW << "\tCannot Stream in Master-Slave: /dev/video" << MASTER_DEVNODE <<
							 		" needs to be Active:\n" << RESET;
	else if (dolMode)
    	    std::cout << tabSpace() << YELLOW << "\t\tSync Disabled in DOL WDR Mode" << RESET << std::endl;

	else if (g_cameraDevices.size() <= MAX_SYNC_CAMERAS) {
	    PROPAGATE_ERROR(masterSlaveHandler.initialize((uint32_t)g_cameraDevices.size()));

	    if (masterSlaveHandler.queryMasterSlaveControl(true, (uint32_t)g_cameraDevices.size())) {
    	    	PROPAGATE_ERROR(masterSlaveHandler.setMasterSlaveMode());
	    	g_masterDev = 1;
	    } else 
		std::cout << tabSpace() << YELLOW << "\t Master-Slave Control not Found. Running in Async Mode" << std::endl;
	    
	} else 
    	    std::cout << tabSpace() << YELLOW << "\tDisabled Master-Slave Mode since Cameras Sync Restricted to " << MAX_SYNC_CAMERAS << " Cameras:\n" << RESET;
	
    	drawLine();
    } 

    // Display The Streaming Resolution && Argus Version
    std::cout << tabSpace() << BLUE << "Argus Version:\t" << RESET;
    std::cout << tabSpace() << YELLOW << iCameraProvider->getVersion().c_str() 
				<< ":" << ARGUS_VERSION << std::endl;
    drawLine();
    std::cout << tabSpace() << BLUE << "Resolution:\t" << RESET;
    std::cout << tabSpace() << YELLOW << iSensorMode->getResolution().width() << " x " 
				<< iSensorMode->getResolution().height() << " @ "
				<< TimeValue::fromNSec(iSensorMode->getFrameDurationRange().min()).toCyclesPerSec() 
				<< " fps";
    if (dolMode)
	std::cout << " : DOL 2-Frame" << std::endl;
    else
	std::cout << std::endl;
    drawLine();

    // Stream Only the User Required Cameras
    uint32_t streamCount = g_cameraDevices.size() < MAX_CAMERA_NUM ?
            g_cameraDevices.size() : MAX_CAMERA_NUM;

    if (streamCount > options.numCameraDevices())
	streamCount = options.numCameraDevices();

    // Initialize Master Device first if enabled
    if (g_masterDev) {
	cameraSessions[0].reset(iCameraProvider->createCaptureSession(g_cameraDevices[MASTER_DEVNODE]));
        captureHolders[0].reset(new CameraModulesEGL(cameraSessions[0].get()));
        if (!captureHolders[0].get()->initialize(g_cameraDevices[MASTER_DEVNODE],sensormodes[options.sensorModeIndex()]))
            ORIGINATE_ERROR("Failed to initialize Camera session %d", MASTER_DEVNODE);

	cameraModules.push_back(captureHolders[0].get());
    }

    // Intialize Camera Device with Output Stream
    for (uint32_t i = 0, j = 1; i < streamCount; i++, j++)
    {
	if (g_masterDev) {
	    if (i == MASTER_DEVNODE) {
		--j;
		continue;
	    }
	    cameraSessions[j].reset(iCameraProvider->createCaptureSession(g_cameraDevices[i]));
            captureHolders[j].reset(new CameraModulesEGL(cameraSessions[j].get()));
            if (!captureHolders[j].get()->initialize(g_cameraDevices[i],sensormodes[options.sensorModeIndex()]))
		ORIGINATE_ERROR("Failed to initialize Camera session %d", j);

	    cameraModules.push_back(captureHolders[j].get());

	} else { 
	    cameraSessions[i].reset(iCameraProvider->createCaptureSession(g_cameraDevices[i]));
            captureHolders[i].reset(new CameraModulesEGL(cameraSessions[i].get()));
            if (!captureHolders[i].get()->initialize(g_cameraDevices[i],sensormodes[options.sensorModeIndex()]))
		ORIGINATE_ERROR("Failed to initialize Camera session %d", i);

	    cameraModules.push_back(captureHolders[i].get());
	}
    }

    // Process thread to handle all operations such as capture, video record, sync, stream metadata
    ProcessFrameThread processConsumer(cameraModules, Rectangle<uint32_t>(0,0,displayWidth,displayHeight), m_renderer);
    PROPAGATE_ERROR(processConsumer.initialize());
    PROPAGATE_ERROR(processConsumer.waitRunning());

    // interface for terminal input and controls
    CommandInterface userIOEngine(cameraModules);
    PROPAGATE_ERROR(userIOEngine.Initialize());
    
    // Submit capture requests.
    for (uint32_t i = 0; i < streamCount; i++) {
	ICaptureSession *iPreviewCaptureSession = interface_cast<ICaptureSession>(captureHolders[i].get()->getSession());
	// Start the preview
	if (iPreviewCaptureSession->repeat(captureHolders[i].get()->getRequest()) != STATUS_OK)
	    ORIGINATE_ERROR("Failed to start repeat capture request for preview");
    }

    // Main Command Interface Function
    while(g_streamState) {
	signal(SIGINT, signal_handler);
        PROPAGATE_ERROR(userIOEngine.UserCommandExecute());
    }

    // Wait for the consumer threads to complete.
    PROPAGATE_ERROR(processConsumer.shutdown());
    PROPAGATE_ERROR(userIOEngine.Shutdown());

    // Stop Camera capture Requests
    PRODUCER_PRINT("Stopping all repeat Capture Requests\n");
    for (int32_t i = streamCount-1; i >= 0 ; i--) {
	    PRODUCER_PRINT("Stopping Device [%d] streaming\n",i);
	    ICaptureSession *iPreviewCaptureSession = interface_cast<ICaptureSession>(captureHolders[i].get()->getSession());
	    // all done shut down
	    iPreviewCaptureSession->stopRepeat();
	    while(iPreviewCaptureSession->isRepeating()) {
	        PRODUCER_PRINT("Still streaming - cam_%d\n",i);
	    }
	    iPreviewCaptureSession->waitForIdle();
    }
    // Destroy the capture resources.
    for (int32_t i = streamCount-1; i >= 0 ; i--)
    {
	PRODUCER_PRINT("Closing Camera [%d]\n",i);
        captureHolders[i].reset();
	cameraSessions[i].reset();
    }

    // Change all Cameras to Master Mode Before Closing
    masterSlaveHandler.changeMasterSlaveMode(false);
    PROPAGATE_ERROR(masterSlaveHandler.initialize((uint32_t)g_cameraDevices.size()));
    if (masterSlaveHandler.queryMasterSlaveControl(false, (uint32_t)g_cameraDevices.size()))
	PROPAGATE_ERROR(masterSlaveHandler.setMasterSlaveMode());

    // Shut down Argus.
    g_cameraProvider.reset();

    // Cleanup the EGL display
    if (options.previewMode())
	delete m_renderer;

    PRODUCER_PRINT("Done -- exiting.\n");

    return true;
}

}; // namespace ArgusSamples

int main(int argc, char** argv)
{
    ArgusSamples::MultiCameraAppOptions options(basename(argv[0]));
    if (!options.parse(argc, argv))
        return EXIT_FAILURE;
    if (options.requestedExit())
        return EXIT_SUCCESS;

    if (!ArgusSamples::execute(options))
        return EXIT_FAILURE;

    return EXIT_SUCCESS;
}
