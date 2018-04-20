/*
 *  calc.cpp
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
 *  Copyright 2012-2015 ARToolworks, Inc. All Rights Reserved.
 *
 *  Author(s): Philip Lamb, Hirokazu Kato
 *
 */

#include "calc.hpp"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core_c.h>

static void convParam(const float intr[3][4], const float dist[AR_DIST_FACTOR_NUM_MAX], const int xsize, const int ysize, const int dist_function_version, ARParam *param);
static ARdouble getSizeFactor(ARdouble const dist_factor[AR_DIST_FACTOR_NUM_MAX], const int xsize, const int ysize, const int dist_function_version);

static void calcChessboardCorners(const Calibration::CalibrationPatternType patternType, cv::Size patternSize, float patternSpacing, std::vector<cv::Point3f>& corners)
{
    corners.resize(0);
    
    switch (patternType) {
        case Calibration::CalibrationPatternType::CHESSBOARD:
        case Calibration::CalibrationPatternType::CIRCLES_GRID:
            for (int j = 0; j < patternSize.height; j++)
                for (int i = 0; i < patternSize.width; i++)
                    corners.push_back(cv::Point3f(float(i*patternSpacing), float(j*patternSpacing), 0));
            break;
            
        case Calibration::CalibrationPatternType::ASYMMETRIC_CIRCLES_GRID:
            for (int j = 0; j < patternSize.height; j++)
                for (int i = 0; i < patternSize.width; i++)
                    corners.push_back(cv::Point3f(float((2*i + j % 2)*patternSpacing), float(j*patternSpacing), 0));
            break;
            
        default:
            ARLOGe("Unknown pattern type.\n");
    }
}

void calc(const int capturedImageNum,
          const Calibration::CalibrationPatternType patternType,
          const cv::Size patternSize,
		  const float patternSpacing,
		  const std::vector<std::vector<cv::Point2f> >& cornerSet,
		  const int width,
		  const int height,
          const int dist_function_version,
		  ARParam *param_out,
		  ARdouble *err_min_out,
		  ARdouble *err_avg_out,
		  ARdouble *err_max_out)
{
    if (dist_function_version != 5 && dist_function_version != 4) {
        ARLOGe("Unsupported distortion function version %d.\n", dist_function_version);
        return;
    }

    // Set version.
    int flags = 0;
    cv::Mat distortionCoeff;
    if (dist_function_version == 5) {
        distortionCoeff = cv::Mat::zeros(12, 1, CV_64F);
        flags |= cv::CALIB_RATIONAL_MODEL|cv::CALIB_THIN_PRISM_MODEL;
    } else /* dist_function_version == 4 */ {
        distortionCoeff = cv::Mat::zeros(5, 1, CV_64F);
        flags |= cv::CALIB_FIX_K3;
    }
    double aspectRatio = 1.0;

    // Set up object points.
    std::vector<std::vector<cv::Point3f> > objectPoints(1);
    calcChessboardCorners(patternType, patternSize, patternSpacing, objectPoints[0]);
    objectPoints.resize(capturedImageNum, objectPoints[0]);
        
    cv::Mat intrinsics = cv::Mat::eye(3, 3, CV_64F);
    if (flags & cv::CALIB_FIX_ASPECT_RATIO)
       intrinsics.at<double>(0,0) = aspectRatio;
    
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;
    
    double rms = calibrateCamera(objectPoints, cornerSet, cv::Size(width, height), intrinsics,
                                 distortionCoeff, rotationVectors, translationVectors, flags);
    
    ARLOGi("RMS error reported by calibrateCamera: %g\n", rms);
    
    bool ok = checkRange(intrinsics) && checkRange(distortionCoeff);
    if (!ok) ARLOGe("cv::checkRange(intrinsics) && cv::checkRange(distortionCoeff) reported not OK.\n");
    
    
    float           intr[3][4];
    float           dist[AR_DIST_FACTOR_NUM_MAX];
    ARParam         param;
    int i, j, k;

    for (j = 0; j < 3; j++) {
        for (i = 0; i < 3; i++) {
            intr[j][i] =  (float)intrinsics.at<double>(j, i);
        }
        intr[j][3] = 0.0f;
    }
    if (dist_function_version == 5) {
        for (i = 0; i < 12; i++) dist[i] = (float)distortionCoeff.at<double>(i);
    } else /* dist_function_version == 4 */ {
        for (i = 0; i < 4; i++) dist[i] = (float)distortionCoeff.at<double>(i);
    }
    convParam(intr, dist, width, height, dist_function_version, &param);
    arParamDisp(&param);

    CvMat          *rotationVector;
    CvMat          *rotationMatrix;
    double          trans[3][4];
    ARdouble        cx, cy, cz, hx, hy, h, sx, sy, ox, oy, err;
    ARdouble        err_min = 1000000.0f, err_avg = 0.0f, err_max = 0.0f;
    rotationVector     = cvCreateMat(1, 3, CV_32FC1);
    rotationMatrix     = cvCreateMat(3, 3, CV_32FC1);

    for (k = 0; k < capturedImageNum; k++) {
        for (i = 0; i < 3; i++) {
            ((float *)(rotationVector->data.ptr))[i] = (float)rotationVectors.at(k).at<double>(i);
        }
        cvRodrigues2(rotationVector, rotationMatrix, 0);
        for (j = 0; j < 3; j++) {
            for (i = 0; i < 3; i++) {
                trans[j][i] = ((float *)(rotationMatrix->data.ptr + rotationMatrix->step*j))[i];
            }
            trans[j][3] = (float)translationVectors.at(k).at<double>(j);
        }
        //arParamDispExt(trans);

        err = 0.0;
        for (i = 0; i < patternSize.width; i++) {
            for (j = 0; j < patternSize.height; j++) {
                float x = objectPoints[0][i * patternSize.height + j].x;
                float y = objectPoints[0][i * patternSize.height + j].y;
                cx = trans[0][0] * x + trans[0][1] * y + trans[0][3];
                cy = trans[1][0] * x + trans[1][1] * y + trans[1][3];
                cz = trans[2][0] * x + trans[2][1] * y + trans[2][3];
                hx = param.mat[0][0] * cx + param.mat[0][1] * cy + param.mat[0][2] * cz + param.mat[0][3];
                hy = param.mat[1][0] * cx + param.mat[1][1] * cy + param.mat[1][2] * cz + param.mat[1][3];
                h  = param.mat[2][0] * cx + param.mat[2][1] * cy + param.mat[2][2] * cz + param.mat[2][3];
                if (h == 0.0) continue;
                sx = hx / h;
                sy = hy / h;
                arParamIdeal2Observ(param.dist_factor, sx, sy, &ox, &oy, param.dist_function_version);
                sx = (ARdouble)cornerSet[k][i * patternSize.height + j].x;
                sy = (ARdouble)cornerSet[k][i * patternSize.height + j].y;
                err += (ox - sx)*(ox - sx) + (oy - sy)*(oy - sy);
            }
        }
        err = sqrtf(err/(patternSize.width*patternSize.height));
        ARPRINT("Err[%2d]: %f[pixel]\n", k + 1, err);

        // Track min, avg, and max error.
        if (err < err_min) err_min = err;
        err_avg += err;
        if (err > err_max) err_max = err;
    }
    err_avg /= (ARdouble)capturedImageNum;
    *err_min_out = err_min;
    *err_avg_out = err_avg;
    *err_max_out = err_max;

    *param_out = param;

    cvReleaseMat(&rotationVector);
    cvReleaseMat(&rotationMatrix);
}

static void convParam(const float intr[3][4], const float dist[AR_DIST_FACTOR_NUM_MAX], const int xsize, const int ysize, const int dist_function_version, ARParam *param)
{
    double   s;
    int      i, j;
    
    if (dist_function_version != 5 && dist_function_version != 4) {
        ARLOGe("Unsupported distortion function version %d.\n", dist_function_version);
        return;
    }
    
	param->dist_function_version = dist_function_version;
    param->xsize = xsize;
    param->ysize = ysize;

    param->dist_factor[0] = (ARdouble)dist[0];         /* k1  */
    param->dist_factor[1] = (ARdouble)dist[1];         /* k2  */
    param->dist_factor[2] = (ARdouble)dist[2];         /* p1  */
    param->dist_factor[3] = (ARdouble)dist[3];         /* p2  */
    if (dist_function_version == 5) {
        param->dist_factor[4] = (ARdouble)dist[4];     /* k3  */
        param->dist_factor[5] = (ARdouble)dist[5];     /* k4  */
        param->dist_factor[6] = (ARdouble)dist[6];     /* k5  */
        param->dist_factor[7] = (ARdouble)dist[7];     /* k6  */
        param->dist_factor[8] = (ARdouble)dist[8];     /* s1  */
        param->dist_factor[9] = (ARdouble)dist[9];     /* s2  */
        param->dist_factor[10] = (ARdouble)dist[10];   /* s3  */
        param->dist_factor[11] = (ARdouble)dist[11];   /* s4  */
        param->dist_factor[12] = (ARdouble)intr[0][0]; /* fx  */
        param->dist_factor[13] = (ARdouble)intr[1][1]; /* fy  */
        param->dist_factor[14] = (ARdouble)intr[0][2]; /* cx  */
        param->dist_factor[15] = (ARdouble)intr[1][2]; /* cy  */
        param->dist_factor[16] = (ARdouble)1.0;        /* s   */
    } else /* dist_function_version == 4 */ {
        param->dist_factor[4] = (ARdouble)intr[0][0];  /* fx  */
        param->dist_factor[5] = (ARdouble)intr[1][1];  /* fy  */
        param->dist_factor[6] = (ARdouble)intr[0][2];  /* cx  */
        param->dist_factor[7] = (ARdouble)intr[1][2];  /* cy  */
        param->dist_factor[8] = (ARdouble)1.0;         /* s   */
    }

    for (j = 0; j < 3; j++) {
        for (i = 0; i < 4; i++) {
            param->mat[j][i] = (ARdouble)intr[j][i];
        }
    }

    s = getSizeFactor(param->dist_factor, xsize, ysize, param->dist_function_version);
    param->mat[0][0] /= s;
    param->mat[0][1] /= s;
    param->mat[1][0] /= s;
    param->mat[1][1] /= s;
    if (dist_function_version == 5) {
        param->dist_factor[16] = s;
    } else /* dist_function_version == 4 */ {
        param->dist_factor[8] = s;
    }
}

static ARdouble getSizeFactor(const ARdouble dist_factor[AR_DIST_FACTOR_NUM_MAX], const int xsize, const int ysize, const int dist_function_version)
{
    ARdouble  ox, oy, ix, iy;
    ARdouble  olen, ilen;
    ARdouble  sf, sf1;
    ARdouble  cx, cy;
    
    if (dist_function_version == 5) {
        cx = dist_factor[14];
        cy = dist_factor[15];
    } else if (dist_function_version == 4) {
        cx = dist_factor[6];
        cy = dist_factor[7];
    } else {
        ARLOGe("Unsupported distortion function version %d.\n", dist_function_version);
        return 1.0;
    }

    sf = 100.0f;

    ox = 0.0f;
    oy = cy;
    olen = cx;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = cx - ix;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }

    ox = xsize;
    oy = cy;
    olen = xsize - cx;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = ix - cx;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }

    ox = cx;
    oy = 0.0;
    olen = cy;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = cy - iy;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }

    ox = cx;
    oy = ysize;
    olen = ysize - cy;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = iy - cy;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }


    ox = 0.0f;
    oy = 0.0f;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = cx - ix;
    olen = cx;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }
    ilen = cy - iy;
    olen = cy;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }

    ox = xsize;
    oy = 0.0f;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = ix - cx;
    olen = xsize - cx;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }
    ilen = cy - iy;
    olen = cy;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }

    ox = 0.0f;
    oy = ysize;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = cx - ix;
    olen = cx;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }
    ilen = iy - cy;
    olen = ysize - cy;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }

    ox = xsize;
    oy = ysize;
    arParamObserv2Ideal(dist_factor, ox, oy, &ix, &iy, dist_function_version);
    ilen = ix - cx;
    olen = xsize - cx;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }
    ilen = iy - cy;
    olen = ysize - cy;
    //ARPRINT("Olen = %f, Ilen = %f, s = %f\n", olen, ilen, ilen / olen);
    if (ilen > 0.0f) {
        sf1 = ilen / olen;
        if (sf1 < sf) sf = sf1;
    }

    if (sf == 100.0f) sf = 1.0f;

    return sf;
}

