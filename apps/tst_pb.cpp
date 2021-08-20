///  @file subStream.cpp
///
///  @brief This example illustrates how to record and play back a bltstream file.
///
///  @author Birgit Hasenberger, Alex Falkensteiner
///
///  For the API User Manual, the API Reference Manual, and further support, visit 
///  http://support.becom-group.com/wiki/Blt_ToF_API.


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <bta.h>

#ifdef PLAT_WINDOWS
#include <windows.h> 
#elif defined PLAT_LINUX
#include <unistd.h>
#include <time.h>
#else
#error "No platform defined"
#endif

static uint32_t BTAgetTickCount();
static void wait(int ms);
static void errorHandling(BTA_Status status);
static uint32_t hours, min, sec, msec, musec;
static uint32_t timeArr;


// Example for an implementation of the frameArrivedEx callback handler (more details in 
// example "Advanced Configuration Callbacks")
//----------------------------------------------------------------------------------------------
static void BTA_CALLCONV frameArrivedEx(BTA_Handle handle, BTA_Frame *frame) {
	timeArr = frame->timeStamp;
	musec = timeArr % 1000;
	timeArr /= 1000;
	msec = timeArr % 1000;
	timeArr /= 1000;
	sec = timeArr % 60;
	timeArr /= 60;
	min = timeArr % 60;
	hours = timeArr / 60;
	printf("Frame arrived: Frame no. %d at %02d:%02d:%02d.%03d %03d.\n", frame->frameCounter, hours, min, sec, msec, musec);
	return;
}

// Example for an implementation of the infoEventEx callback handler (more details in example 
// "Advanced Configuration Callbacks")
//----------------------------------------------------------------------------------------------
static void BTA_CALLCONV infoEventEx(BTA_Handle handle, BTA_EventId eventId, int8_t *msg) {
	char statusString[100];
	BTAstatusToString(eventId, statusString, sizeof(statusString));
	printf(" %50.2f: infoEventEx: handle: 0x%p  (%s) %s\n", BTAgetTickCount() / 1000.0, handle, statusString, msg);
}

int main(int argc, char* argv[]) {

	// Purpose of this example
	//----------------------------------------------------------------------------------------------
	// In this example, frames delivered by the device are written to the disk and afterwards 
	// played back. Very conveniently, all the frame data is stored in a *.bltstream file. These 
	// files can be replayed by the BltTofApi or the BltTofSuite as if the same sensor were 
	// connected.

	BTA_Status status;

	// Playback of previously grabbed bltstream file
	//----------------------------------------------------------------------------------------------
	// A frameArrived callback handler is registered for the purpose of this playback example.

	printf("\n");
	BTA_Config config2;
	BTAinitConfig(&config2);
	config2.deviceType = BTA_DeviceTypeBltstream;
	config2.bltstreamFilename = (uint8_t *)argv[1]; // (ASCII coded)
	config2.infoEventEx = &infoEventEx;
	config2.verbosity = 5;
	config2.frameArrivedEx = &frameArrivedEx;
	
	// After configuration, the connection to the bltstream file can be established.
	BTA_Handle handleBltstream;
	status = BTAopen(&config2, &handleBltstream);
	errorHandling(status);
	
	// The BTAgetLibParam and BTAsetLibParam functions allow the user to obtain information about 
	// stream and control the playback.
	float totalFrameCount;
	status = BTAgetLibParam(handleBltstream, BTA_LibParamStreamTotalFrameCount, &totalFrameCount);
	errorHandling(status);
	printf("The bltstream beeing played back contains a total of %0.0f frames\n", totalFrameCount);
	
	printf("Starting playback by setting BTA_LibParamStreamAutoPlaybackSpeed=1\n");
	BTAsetLibParam(handleBltstream, BTA_LibParamStreamAutoPlaybackSpeed, 1);
	// The stream is played back in background and the frames are delivered via the callback.
	// Alternatively, if queueing is enabled, one can also call BTAgetFrame

	wait(1000);
	
	printf("Setting the playback speed to factor 3 by setting BTA_LibParamStreamAutoPlaybackSpeed=3\n");
	BTAsetLibParam(handleBltstream, BTA_LibParamStreamAutoPlaybackSpeed, 3);
	errorHandling(status);

	wait(3000);
	
	printf("\n");
	printf("BTAclose()\n");
	status = BTAclose(&handleBltstream);
	errorHandling(status);
	return 0;
}


static void errorHandling(BTA_Status status) {
	if (status != BTA_StatusOk) {
		char statusString[100];
		BTAstatusToString(status, statusString, (uint16_t)sizeof(statusString));
		printf("error: %s (%d)\n", statusString, status);
		printf("Hit <Return> to end the example\n");
		fgetc(stdin);
		exit(0);
	}
}

static uint32_t BTAgetTickCount() {
#   ifdef PLAT_WINDOWS
	return GetTickCount();
#   elif defined PLAT_LINUX
	struct timespec ts;
	if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
		// what can I do?
		return 0;
	}
	return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
#   endif
}

static void wait(int ms) {
#   if defined(PLAT_LINUX) || defined(linux)
	usleep(ms * 1000);
#   elif defined(PLAT_WINDOWS) || defined(WIN32) || defined(WIN64)
	Sleep(ms);
#   endif
}
