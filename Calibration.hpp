/*
 *  Calibration.hpp
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

#pragma once

#include <ARX/AR/ar.h>
#include <opencv2/core/core.hpp>
#include <ARX/ARVideoSource.h>
#include <map>

#include <ARX/ARUtil/thread_sub.h>

class Calibration
{
public:
    
    enum class CalibrationPatternType {
        CHESSBOARD,
        CIRCLES_GRID,
        ASYMMETRIC_CIRCLES_GRID
    };
    
    static std::map<CalibrationPatternType, cv::Size> CalibrationPatternSizes;
    static std::map<CalibrationPatternType, float> CalibrationPatternSpacings;
    
    /*!
        @brief Create a new calibration session.
        @param patternType The calibration pattern that will be used.
        @param calibImageCountMax The maximum number of images of the calibration pattern to capture.
        @param patternSize The size of the calibration pattern. For the chessboard, this is the number
            of rows minus 1 x the number of columns minus 1. For the assymetric circles grid, this is
            the number of circles in each row x (the number of non-offset rows + the number of offset rows).
        @param chessboardSquareWidth The pattern spacing. For the chessboard, this is the width of each square.
            For the assymetric circles grid, this is the spacing between centres of adjacent columsn divided
            by 2.
        @param videoWidth The width of video frames that will be passed to the frame() method.
        @param videoHeight The height of video frames that will be passed to the frame() method.
     */
    Calibration(const CalibrationPatternType patternType, const int calibImageCountMax, const cv::Size patternSize, const int chessboardSquareWidth, const int videoWidth, const int videoHeight);
    
    /*!
        @brief Get the number of calibration patterns captured so far.
     */
    int calibImageCount() const {return (int)m_corners.size(); }
    
    /*!
        @brief Get the number of calibration patterns to be captured.
     */
    int calibImageCountMax() const {return m_calibImageCountMax; }
    
    /*!
        @brief Pass a video frame for possible processing.
        @details The first step in processing is searching the video frame for the calibration pattern
            corners ("corner finding"). This process can take anywhere from milliseconds to several seconds
            per frame, and runs in a separate thread. If the corner finder is waiting for a frame, this
            function will copy the source frame, and begin corner finding.
        @param vs ARVideoSource from which to grab the frame.
        @result true if the frame was processed OK, false in the case of error.
     */
    bool frame(ARVideoSource *vs);
    
    /*!
        @brief Access the results of the most recent corner finding processing step, with lock.
        @details This function gives access to the results of the most recent corner finding processing
            allowing, for example, visual feedback to the user of corner locations.
            This call locks the results from further updates until cornerFinderResultsUnlock() is called,
            so the user should copy the results if long-term access is required.
        @param cornerFoundAllFlag If non-NULL, the int pointed to will be set to 1 if all corners
            were found, or 0 if some or no corners were found.
        @param corners Corner locations, in screen coordinates.
        @param videoFrame Pointer, which will be set to point to the raw video frame in which the corners
            were found.
        @result true if the corners were found in the most recent processing step, false otherwise.
     */
    bool cornerFinderResultsLockAndFetch(int *cornerFoundAllFlag, std::vector<cv::Point2f>& corners, ARUint8** videoFrame);
    
    /*!
        @brief Unlock the results of the most recent corner finding processing step.
        @details Must be called after calling cornerFinderResultsLockAndFetch to allow further corner
            finding to proceed.
        @result true if the results were unlocked OK, false in the case of error.
     */
    bool cornerFinderResultsUnlock(void);
    
    /*!
        @brief Capture the most recent corner finder results as a calibration input.
     */
    bool capture();
    
    /*!
        @brief Undo the capture of the most recent corner finder results.
     */
    bool uncapture();
    
    /*!
        @brief Discard all captured corner finder results.
     */
    bool uncaptureAll();
    
    /*!
        @brief Perform a calibration calculation on the currently captured results, and return as an ARParam.
        @param param_out Pointer to an ARParam which will be filled with the calibration result.
        @param err_min_out Pointer to an ARdouble which will be filled with the minimum reprojection error in the set of captured calibration patterns.
        @param err_avg_out Pointer to an ARdouble which will be filled with the average reprojection error in the set of captured calibration patterns.
        @param err_max_out Pointer to an ARdouble which will be filled with the maximum reprojection error in the set of captured calibration patterns.
     */
    void calib(ARParam *param_out, ARdouble *err_min_out, ARdouble *err_avg_out, ARdouble *err_max_out);
    
    /*!
        @brief Terminate calibration and cleanup.
     */
    ~Calibration();
    
private:
    
    Calibration(const Calibration&) = delete; // No copy construction.
    Calibration& operator=(const Calibration&) = delete; // No copy assignment.
    
    // This function runs the heavy-duty corner finding process on a secondary thread. Must be static so it can be
    // passed to threadInit().
    static void *cornerFinder(THREAD_HANDLE_T *threadHandle);
    
    // A class to encapsulate the inputs and outputs of a corner-finding run, and to allow for copying of the results
    // of a completed run.
    class CalibrationCornerFinderData {
    public:
        CalibrationCornerFinderData(const CalibrationPatternType patternType_in, const cv::Size patternSize_in, const int videoWidth_in, const int videoHeight_in);
        CalibrationCornerFinderData(const CalibrationCornerFinderData& orig);
        const CalibrationCornerFinderData& operator=(const CalibrationCornerFinderData& orig);
        ~CalibrationCornerFinderData();
        CalibrationPatternType patternType;
        cv::Size             patternSize;
        int                  videoWidth;
        int                  videoHeight;
        uint8_t             *videoFrame;
        IplImage            *calibImage;
        int                  cornerFoundAllFlag;
        std::vector<cv::Point2f> corners;
    private:
        void init();
        void copy(const CalibrationCornerFinderData& orig);
        void dealloc();
    };
    
    CalibrationCornerFinderData m_cornerFinderData; // Corner finder input and output.
    THREAD_HANDLE_T     *m_cornerFinderThread = NULL;
    pthread_mutex_t      m_cornerFinderResultLock;
    CalibrationCornerFinderData m_cornerFinderResultData; // Corner finder results copy, for display to user.
    
    std::vector<std::vector<cv::Point2f> > m_corners; // Collected corner information which gets passed to the OpenCV calibration function.
    int                  m_calibImageCountMax;
    CalibrationPatternType m_patternType;
    cv::Size             m_patternSize;
    int                  m_chessboardSquareWidth;
    int                  m_videoWidth;
    int                  m_videoHeight;
};
