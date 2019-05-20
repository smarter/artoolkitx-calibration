/*
 *  flow.cpp
 *  artoolkitX
 *
 *  This file is part of artoolkitX.
 *
 *  artoolkitX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  artoolkitX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with artoolkitX.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  As a special exception, the copyright holders of this library give you
 *  permission to link this library with independent modules to produce an
 *  executable, regardless of the license terms of these independent modules, and to
 *  copy and distribute the resulting executable under terms of your choice,
 *  provided that you also meet, for each linked independent module, the terms and
 *  conditions of the license of that module. An independent module is a module
 *  which is neither derived from nor based on this library. If you modify this
 *  library, you may extend this exception to your version of the library, but you
 *  are not obligated to do so. If you do not wish to do so, delete this exception
 *  statement from your version.
 *
 *  Copyright 2018 Realmax, Inc.
 *  Copyright 2015-2017 Daqri LLC. All Rights Reserved.
 *  Copyright 2013-2015 ARToolworks, Inc. All Rights Reserved.
 *
 *  Author(s): Philip Lamb
 *
 */

#ifndef _GNU_SOURCE
#  define _GNU_SOURCE // asprintf()
#endif

#include "flow.hpp"

#include <stdio.h> // asprintf()
#include <pthread.h>
#include <Eden/EdenMessage.h>
#include <ARX/AR/ar.h>

//
// Globals.
//

static bool gInited = false;
static FLOW_STATE gState = FLOW_STATE_NOT_INITED;
static pthread_mutex_t gStateLock;
static pthread_mutex_t gEventLock;
static pthread_cond_t gEventCond;
static EVENT_t gEvent = EVENT_NONE;
static EVENT_t gEventMask = EVENT_NONE;
static pthread_t gThread;
static int gThreadExitStatus;
static bool gStop;

// Completion callback.
static FLOW_CALLBACK_t gCallback = NULL;
static void *gCallbackUserdata = NULL;

// Logging macros
#define  LOG_TAG    "flow"

// Status bar.
#define STATUS_BAR_MESSAGE_BUFFER_LEN 128
unsigned char statusBarMessage[STATUS_BAR_MESSAGE_BUFFER_LEN] = "";

// Calibration inputs.
static Calibration *gFlowCalib = nullptr;


//
// Function prototypes.
//

static void *flowThread(void *arg);
static void flowSetEventMask(const EVENT_t eventMask);

//
// Functions.
//

bool flowInitAndStart(Calibration *calib, FLOW_CALLBACK_t callback, void *callback_userdata)
{
    pthread_mutex_init(&gStateLock, NULL);
    pthread_mutex_init(&gEventLock, NULL);
    pthread_cond_init(&gEventCond, NULL);

    // Calibration inputs.
    gFlowCalib = calib;

    // Completion callback.
    gCallback = callback;
    gCallbackUserdata = callback_userdata;

    gStop = false;
    pthread_create(&gThread, NULL, flowThread, NULL);

    gInited = true;

    return (true);
}

bool flowStopAndFinal()
{
	void *exit_status_p;		 // Pointer to return value from thread, will be filled in by pthread_join().

	if (!gInited) return (false);

	// Request stop and wait for join.
	gStop = true;
#ifndef ANDROID
	pthread_cancel(gThread); // Not implemented on Android.
#endif
#ifdef DEBUG
	ARLOGi("flowStopAndFinal(): Waiting for flowThread() to exit...\n");
#endif
	pthread_join(gThread, &exit_status_p);
#ifdef DEBUG
#  ifndef ANDROID
	ARLOGi("  done. Exit status was %d.\n",((exit_status_p == PTHREAD_CANCELED) ? 0 : *(int *)(exit_status_p))); // Contents of gThreadExitStatus.
#  else
	ARLOGi("  done. Exit status was %d.\n", *(int *)(exit_status_p)); // Contents of gThreadExitStatus.
#  endif
#endif
    
    gFlowCalib = nullptr;

	// Clean up.
	pthread_mutex_destroy(&gStateLock);
	pthread_mutex_destroy(&gEventLock);
	pthread_cond_destroy(&gEventCond);
	gState = FLOW_STATE_NOT_INITED;
	gInited = false;

	return true;
}

FLOW_STATE flowStateGet()
{
	FLOW_STATE ret;

	if (!gInited) return (FLOW_STATE_NOT_INITED);

	pthread_mutex_lock(&gStateLock);
	ret = gState;
	pthread_mutex_unlock(&gStateLock);
	return (ret);
}

static void flowStateSet(FLOW_STATE state)
{
	if (!gInited) return;

	pthread_mutex_lock(&gStateLock);
	gState = state;
	pthread_mutex_unlock(&gStateLock);
}

static void flowSetEventMask(const EVENT_t eventMask)
{
	pthread_mutex_lock(&gEventLock);
	gEventMask = eventMask;
	pthread_mutex_unlock(&gEventLock);
}

bool flowHandleEvent(const EVENT_t event)
{
	bool ret;

	if (!gInited) return false;

	pthread_mutex_lock(&gEventLock);
	if ((event & gEventMask) == EVENT_NONE) {
		ret = false; // not handled (discarded).
	} else {
		gEvent = event;
		pthread_cond_signal(&gEventCond);
		ret = true;
	}
	pthread_mutex_unlock(&gEventLock);

	return (ret);
}

static EVENT_t flowWaitForEvent(void)
{
	EVENT_t ret;

	pthread_mutex_lock(&gEventLock);
	while (gEvent == EVENT_NONE && !gStop) {
#ifdef ANDROID
        // Android "Bionic" libc doesn't implement cancelation, so need to let wait expire somewhat regularly.
        const struct timespec twoSeconds = {2, 0};
        pthread_cond_timedwait_relative_np(&gEventCond, &gEventLock, &twoSeconds);
#else
		pthread_cond_wait(&gEventCond, &gEventLock);
#endif
	}
	ret = gEvent;
	gEvent = EVENT_NONE; // Clear wait state.
	pthread_mutex_unlock(&gEventLock);

	return (ret);
}

static void flowThreadCleanup(void *arg)
{
	pthread_mutex_unlock(&gStateLock);
    // Clear status bar.
    statusBarMessage[0] = '\0';
}

static void *flowThread(void *arg)
{
	bool captureDoneSinceBackButtonLastPressed;
	EVENT_t event;
	// TYPE* TYPE_INSTANCE = (TYPE *)arg; // Cast the thread start arg to the correct type.

    ARLOGi("Start flow thread.\n");

    // Register our cleanup function, with no arg.
	pthread_cleanup_push(flowThreadCleanup, NULL);

	// Welcome.
	flowStateSet(FLOW_STATE_WELCOME);

	while (!gStop) {
		ARLOGi("1\n");
		if (flowStateGet() == FLOW_STATE_WELCOME) {
			ARLOGi((const char *)"Welcome to artoolkitX Camera Calibrator\n(c)2018 Realmax, Inc. & (c)2017 DAQRI LLC.\n\nPress 'space' to begin a calibration run.\n\nPress 'p' for settings and help.\n");
		} else {
			ARLOGi((const char *)"Press 'space' to begin a calibration run.\n\nPress 'p' for settings and help.\n");
		}
		ARLOGi("2\n");
		flowSetEventMask((EVENT_t)(EVENT_TOUCH | EVENT_MODAL));
		event = flowWaitForEvent();
		ARLOGi("3\n");
		if (gStop) break;
        
        if (event == EVENT_MODAL) {
            flowSetEventMask(EVENT_MODAL);
            event = flowWaitForEvent();
            continue;
        } else {
            //EdenMessageHide();
        }

		// Start capturing.
		captureDoneSinceBackButtonLastPressed = false;
		flowStateSet(FLOW_STATE_CAPTURING);
		flowSetEventMask((EVENT_t)(EVENT_TOUCH|EVENT_BACK_BUTTON));

		do {
			snprintf((char *)statusBarMessage, STATUS_BAR_MESSAGE_BUFFER_LEN, "Capturing image %d/%d", gFlowCalib->calibImageCount() + 1, gFlowCalib->calibImageCountMax());
			printf("Capturing image %d/%d\n", gFlowCalib->calibImageCount() + 1, gFlowCalib->calibImageCountMax());
			event = flowWaitForEvent();
			if (gStop) break;
			if (event == EVENT_TOUCH) {

				if (gFlowCalib->capture()) {
			    	captureDoneSinceBackButtonLastPressed = true;
				}

			} else if (event == EVENT_BACK_BUTTON) {

				if (!captureDoneSinceBackButtonLastPressed) {
                    gFlowCalib->uncaptureAll();
                    break;
				} else {
					gFlowCalib->uncapture();
				}
				captureDoneSinceBackButtonLastPressed = false;
			}

		} while (gFlowCalib->calibImageCount() < gFlowCalib->calibImageCountMax());

		// Clear status bar.
		statusBarMessage[0] = '\0';

		if (gFlowCalib->calibImageCount() < gFlowCalib->calibImageCountMax()) {

			flowSetEventMask(EVENT_TOUCH);
            flowStateSet(FLOW_STATE_DONE);
			ARLOGi((const char *)"Calibration canceled\n");
			flowWaitForEvent();
			if (gStop) break;
			//EdenMessageHide();

		} else {
			ARParam param;
			ARdouble err_min, err_avg, err_max;

			flowSetEventMask(EVENT_NONE);
			flowStateSet(FLOW_STATE_CALIBRATING);
			ARLOGi((const char *)"Calculating camera parameters...\n");
			gFlowCalib->calib(&param, &err_min, &err_avg, &err_max);
    		//EdenMessageHide();

            if (gCallback) (*gCallback)(&param, err_min, err_avg, err_max, gCallbackUserdata);
            gFlowCalib->uncaptureAll(); // prepare for next run.

			// Calibration complete. Post results as status.
			flowSetEventMask(EVENT_TOUCH);
			flowStateSet(FLOW_STATE_DONE);
			unsigned char *buf;
			asprintf((char **)&buf, "Camera parameters calculated (error min=%.3f, avg=%.3f, max=%.3f)", err_min, err_avg, err_max);
			printf("Camera parameters calculated (error min=%.3f, avg=%.3f, max=%.3f)\n", err_min, err_avg, err_max);
			EdenMessageShow(buf);
			free(buf);
			flowWaitForEvent();
			if (gStop) break;
			//EdenMessageHide();

		}

		//pthread_testcancel(); // Not implemented on Android.
	} // while (!gStop);
    
	pthread_cleanup_pop(1); // Unlocks gStateLock.

    ARLOGi("End flow thread.\n");

	gThreadExitStatus = 1; // Put the exit status into a global
	return (&gThreadExitStatus); // Pass a pointer to the global as our exit status.
}


