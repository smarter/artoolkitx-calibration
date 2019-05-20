/*
 *  Calibration.cpp
 *  artoolkitX Camera Calibration Utility
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
 *  Copyright 2015-2017 Daqri, LLC.
 *  Copyright 2002-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb
 *
 */

#include "Calibration.hpp"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "calc.hpp"

//
// A class to encapsulate the inputs and outputs of a corner-finding run, and to allow for copying of the results
// of a completed run.
//

Calibration::CalibrationCornerFinderData::CalibrationCornerFinderData(const Calibration::CalibrationPatternType patternType_in, const cv::Size patternSize_in, const int videoWidth_in, const int videoHeight_in) :
    patternType(patternType_in),
    patternSize(patternSize_in),
    videoWidth(videoWidth_in),
    videoHeight(videoHeight_in),
    cornerFoundAllFlag(0),
    corners()
{
    init();
}

// copy constructor.
Calibration::CalibrationCornerFinderData::CalibrationCornerFinderData(const Calibration::CalibrationCornerFinderData& orig) :
    patternType(orig.patternType),
    patternSize(orig.patternSize),
    videoWidth(orig.videoWidth),
    videoHeight(orig.videoHeight),
    cornerFoundAllFlag(orig.cornerFoundAllFlag),
    corners(orig.corners)
{
    init();
    copy(orig);
}

// copy assignement.
const Calibration::CalibrationCornerFinderData& Calibration::CalibrationCornerFinderData::operator=(const Calibration::CalibrationCornerFinderData& orig)
{
    if (this != &orig) {
        dealloc();
        patternType = orig.patternType;
        patternSize = orig.patternSize;
        videoWidth = orig.videoWidth;
        videoHeight = orig.videoHeight;
        cornerFoundAllFlag = orig.cornerFoundAllFlag;
        corners = orig.corners;
        init();
        copy(orig);
    }
    return *this;
}

Calibration::CalibrationCornerFinderData::~CalibrationCornerFinderData()
{
    dealloc();
}

void Calibration::CalibrationCornerFinderData::init()
{
    if (videoWidth > 0 && videoHeight > 0) {
        arMalloc(videoFrame, uint8_t, videoWidth * videoHeight);
        calibImage = cvCreateImageHeader(cvSize(videoWidth, videoHeight), IPL_DEPTH_8U, 1);
        cvSetData(calibImage, videoFrame, videoWidth); // Last parameter is rowBytes.
    } else {
        videoFrame = nullptr;
        calibImage = nullptr;
    }
}

void Calibration::CalibrationCornerFinderData::copy(const CalibrationCornerFinderData& orig)
{
    memcpy(videoFrame, orig.videoFrame, sizeof(uint8_t) * videoWidth * videoHeight);
}

void Calibration::CalibrationCornerFinderData::dealloc()
{
    if (calibImage) cvReleaseImageHeader(&calibImage);
    free(videoFrame);
}


//
// User-facing calibration functions.
//


std::map<Calibration::CalibrationPatternType, cv::Size> Calibration::CalibrationPatternSizes = {
	{Calibration::CalibrationPatternType::CHESSBOARD, cv::Size(7, 5)},
{Calibration::CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID, cv::Size(4, 11)}
};

std::map<Calibration::CalibrationPatternType, float> Calibration::CalibrationPatternSpacings = {
	{Calibration::CalibrationPatternType::CHESSBOARD, 28.5f},
	//{Calibration::CalibrationPatternType::CHESSBOARD, 30.0f},
    {Calibration::CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID, 20.0f}
};

Calibration::Calibration(const CalibrationPatternType patternType, const int calibImageCountMax, const cv::Size patternSize, const int chessboardSquareWidth, const int videoWidth, const int videoHeight) :
    m_cornerFinderData(patternType, patternSize, videoWidth, videoHeight),
    m_cornerFinderResultData(patternType, patternSize, 0, 0),
    m_calibImageCountMax(calibImageCountMax),
    m_patternType(patternType),
    m_patternSize(patternSize),
    m_chessboardSquareWidth(chessboardSquareWidth),
    m_videoWidth(videoWidth),
    m_videoHeight(videoHeight),
    m_corners()
{
    // Spawn the corner finder worker thread.
    m_cornerFinderThread = threadInit(0, (void *)(&m_cornerFinderData), cornerFinder);
    
    pthread_mutex_init(&m_cornerFinderResultLock, NULL);
}

bool Calibration::frame(ARVideoSource *vs)
{
    //
    // Start of main calibration-related cycle.
    //
	//printf("start frame\n");
    
    // First, see if an image has been completely processed.
    if (threadGetStatus(m_cornerFinderThread)) {
        threadEndWait(m_cornerFinderThread); // We know from status above that worker has already finished, so this just resets it
		//printf("end wait\n");

        
        // Copy the results.
		//printf("lock\n");
        pthread_mutex_lock(&m_cornerFinderResultLock); // Results are also read by GL thread, so need to lock before modifying.
        m_cornerFinderResultData = m_cornerFinderData;
        pthread_mutex_unlock(&m_cornerFinderResultLock);
		//printf("unlock\n");
    }
    
    // If corner finder worker thread is ready and waiting, submit the new image.
    if (!threadGetBusyStatus(m_cornerFinderThread)) {
        // As corner finding takes longer than a single frame capture, we need to copy the incoming image
        // so that OpenCV has exclusive use of it. We copy into cornerFinderData->videoFrame which provides
        // the backing for calibImage.
		AR2VideoBufferT *buff = vs->checkoutFrameIfNewerThan({ 0,0 });
        if (buff) {
            //printf("hi %d %d %d %d\n", vs->getVideoWidth(), vs->getVideoHeight(), buff->time.sec, buff->time.usec);
            memcpy(m_cornerFinderData.videoFrame, buff->buffLuma, vs->getVideoWidth()*vs->getVideoHeight());
            vs->checkinFrame();
			//printf("checked\n");
            
            // Kick off a new cycle of the cornerFinder. The results will be collected on a subsequent cycle.
            threadStartSignal(m_cornerFinderThread);
        } else {
            //printf("ho\n");
        }
    }
    
    //
    // End of main calibration-related cycle.
    //
    return true;
}

bool Calibration::cornerFinderResultsLockAndFetch(int *cornerFoundAllFlag, std::vector<cv::Point2f>& corners, ARUint8** videoFrame)
{
    pthread_mutex_lock(&m_cornerFinderResultLock);
    *cornerFoundAllFlag = m_cornerFinderResultData.cornerFoundAllFlag;
    corners = m_cornerFinderResultData.corners;
    *videoFrame = m_cornerFinderResultData.videoFrame;
    return true;
}

bool Calibration::cornerFinderResultsUnlock(void)
{
    pthread_mutex_unlock(&m_cornerFinderResultLock);
    return true;
}

// Worker thread.
// static
void *Calibration::cornerFinder(THREAD_HANDLE_T *threadHandle)
{
//#ifdef DEBUG
    ARLOGi("Start cornerFinder thread.\n");
//#endif
    
    CalibrationCornerFinderData *cornerFinderDataPtr = (CalibrationCornerFinderData *)threadGetArg(threadHandle);
    
    while (threadStartWait(threadHandle) == 0) {
		//printf("pattern start\n");
        switch (cornerFinderDataPtr->patternType) {
            case CalibrationPatternType::CHESSBOARD:
                cornerFinderDataPtr->cornerFoundAllFlag = cv::findChessboardCorners(cv::cvarrToMat(cornerFinderDataPtr->calibImage), cornerFinderDataPtr->patternSize, cornerFinderDataPtr->corners, CV_CALIB_CB_FAST_CHECK|CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
                break;
			default:
				printf("???\n");
				exit(0);
			/*
            case CalibrationPatternType::CIRCLES_GRID:
                cornerFinderDataPtr->cornerFoundAllFlag = cv::findCirclesGrid(cv::cvarrToMat(cornerFinderDataPtr->calibImage), cornerFinderDataPtr->patternSize, cornerFinderDataPtr->corners, cv::CALIB_CB_SYMMETRIC_GRID);
                break;
            case CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID:
                cornerFinderDataPtr->cornerFoundAllFlag = cv::findCirclesGrid(cv::cvarrToMat(cornerFinderDataPtr->calibImage), cornerFinderDataPtr->patternSize, cornerFinderDataPtr->corners, cv::CALIB_CB_ASYMMETRIC_GRID);
                break;*/
        }
		//printf("pattern end\n");
        //ARLOGi("cornerFinderDataPtr->cornerFoundAllFlag=%d.\n", cornerFinderDataPtr->cornerFoundAllFlag);
        threadEndSignal(threadHandle);
    }
    
//#ifdef DEBUG
    ARLOGi("End cornerFinder thread.\n");
//#endif
    return (NULL);
}

bool Calibration::capture()
{
	printf("capture start %d\n", m_corners.size());
    if (m_corners.size() >= m_calibImageCountMax) return false;
   
    bool saved = false;
    
	printf("capture lock\n");
    pthread_mutex_lock(&m_cornerFinderResultLock);
    if (m_cornerFinderResultData.cornerFoundAllFlag) {
        // Refine the corner positions.
		printf("Refining\n");
        cornerSubPix(cv::cvarrToMat(m_cornerFinderResultData.calibImage), m_cornerFinderResultData.corners, cv::Size(5,5), cvSize(-1,-1), cv::TermCriteria(CV_TERMCRIT_ITER, 100, 0.1));
        
        // Save the corners.
        m_corners.push_back(m_cornerFinderResultData.corners);
        saved = true;
    }
    pthread_mutex_unlock(&m_cornerFinderResultLock);

    if (saved) {
        ARPRINT("---------- %2d/%2d -----------\n", (int)m_corners.size(), m_calibImageCountMax);
        const std::vector<cv::Point2f>& corners = m_corners.back();
        for (std::vector<cv::Point2f>::const_iterator it = corners.begin(); it < corners.end(); it++) {
            ARPRINT("  %f, %f\n", it->x, it->y);
        }
        ARPRINT("---------- %2d/%2d -----------\n", (int)m_corners.size(), m_calibImageCountMax);
    }
    
    return (saved);
}

bool Calibration::uncapture(void)
{
    if (m_corners.size() <= 0) return false;
    m_corners.pop_back();
    return true;
}

bool Calibration::uncaptureAll(void)
{
    if (m_corners.size() <= 0) return false;
    m_corners.clear();
    return true;
}

void Calibration::calib(ARParam *param_out, ARdouble *err_min_out, ARdouble *err_avg_out, ARdouble *err_max_out)
{
    calc((int)m_corners.size(), m_patternType, m_patternSize, m_chessboardSquareWidth, m_corners, m_videoWidth, m_videoHeight, AR_DIST_FUNCTION_VERSION_DEFAULT, param_out, err_min_out, err_avg_out, err_max_out);
}

Calibration::~Calibration()
{
    pthread_mutex_destroy(&m_cornerFinderResultLock);
    
    // Clean up the corner finder.
    if (m_cornerFinderThread) {
        
        threadWaitQuit(m_cornerFinderThread);
        threadFree(&m_cornerFinderThread);
    }
    
    // Calibration input cleanup.
}


