/*
 *  calib_camera.cpp
 *  artoolkitX
 *
 *  Camera calibration utility.
 *
 *  Run with "--help" parameter to see usage.
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
 *  Copyright 2015-2016 Daqri, LLC.
 *  Copyright 2002-2015 ARToolworks, Inc.
 *
 *  Author(s): Hirokazu Kato, Philip Lamb
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#  include <windows.h>
#  define MAXPATHLEN MAX_PATH
#  include <direct.h> // getcwd
#else
#  include <sys/param.h> // MAXPATHLEN
#  include <unistd.h> // getcwd
#endif
#include <ARX/AR/ar.h>
//#include <ARX/ARVideo/video.h>
#include <ARX/ARVideoSource.h>
#include <ARX/ARVideoView.h>
#include <ARX/ARUtil/system.h>
#include <ARX/ARUtil/thread_sub.h>
#include <ARX/ARUtil/time.h>
#include <ARX/ARUtil/file_utils.h>
#include <ARX/ARG/arg.h>
#include <ARX/ARG/mtx.h>
#include <ARX/ARG/shader_gl.h>
#include <ARX/ARG/glStateCache2.h>

#include "fileUploader.h"
#include "Calibration.hpp"
#include "flow.hpp"
#include "Eden/EdenMessage.h"
#include "Eden/EdenGLFont.h"

#include "prefs.hpp"


#include "calib_camera.h"

#ifdef __APPLE__
#  if (HAVE_GL || HAVE_GL3)
#    include <SDL2/SDL_opengl.h>
#  elif HAVE_GLES2
#    include <SDL2/SDL_opengles2.h>
#  endif
#else
#  if (HAVE_GL || HAVE_GL3)
#    include "SDL2/SDL_opengl.h"
#  elif HAVE_GLES2
#    include "SDL2/SDL_opengles2.h"
#  endif
#endif


// ============================================================================
//	Types
// ============================================================================

// ============================================================================
//	Constants
// ============================================================================

#if HAVE_GLES2
// Indices of GL ES program uniforms.
enum {
    UNIFORM_MODELVIEW_PROJECTION_MATRIX,
    UNIFORM_COLOR,
    UNIFORM_COUNT
};
// Indices of of GL ES program attributes.
enum {
    ATTRIBUTE_VERTEX,
    ATTRIBUTE_COUNT
};
#endif // HAVE_GLES2

#define      CHESSBOARD_CORNER_NUM_X        7
#define      CHESSBOARD_CORNER_NUM_Y        5
#define      CHESSBOARD_PATTERN_WIDTH      30.0
#define      CALIB_IMAGE_NUM               10
#define      SAVE_FILENAME                 "camera_para.dat"

// Data upload.
#define QUEUE_DIR "queue"
#define QUEUE_INDEX_FILE_EXTENSION "upload"



#ifdef __APPLE__
#  include <CommonCrypto/CommonDigest.h>
#  define MD5 CC_MD5
#  define MD5_DIGEST_LENGTH CC_MD5_DIGEST_LENGTH
#  define MD5_COUNT_t CC_LONG
#else
//#include <openssl/md5.h>
// Rather than including full OpenSSL header tree, just provide prototype for MD5().
// Usage is here: https://www.openssl.org/docs/manmaster/man3/MD5.html .
#  define MD5_DIGEST_LENGTH 16
#  define MD5_COUNT_t size_t
#  ifdef __cplusplus
extern "C" {
#  endif
unsigned char *MD5(const unsigned char *d, size_t n, unsigned char *md);
#  ifdef __cplusplus
}
#  endif
#endif


#define FONT_SIZE 18.0f
#define UPLOAD_STATUS_HIDE_AFTER_SECONDS 9.0f

// ============================================================================
//	Global variables.
// ============================================================================

// Prefs.
static void *gPreferences = NULL;
Uint32 gSDLEventPreferencesChanged = 0;
static char *gPreferenceCameraOpenToken = NULL;
static char *gPreferenceCameraResolutionToken = NULL;
static bool gCalibrationSave = true;
static char *gCalibrationSaveDir = NULL;
static char *gCalibrationServerUploadURL = NULL;
static char *gCalibrationServerAuthenticationToken = NULL;
static int gPreferencesCalibImageCountMax = CALIB_IMAGE_NUM;
static Calibration::CalibrationPatternType gCalibrationPatternType;
static cv::Size gCalibrationPatternSize;
static float gCalibrationPatternSpacing;

//
// Calibration.
//

static Calibration *gCalibration = nullptr;

//
// Data upload.
//

static char *gFileUploadQueuePath = NULL;
FILE_UPLOAD_HANDLE_t *fileUploadHandle = NULL;

// Video acquisition and rendering.
static ARVideoSource *vs = nullptr;
static ARVideoView *vv = nullptr;
static bool gPostVideoSetupDone = false;
static bool gCameraIsFrontFacing = false;
static long gFrameCount = 0;

// Window and GL context.
static ARG_API drawAPI = ARG_API_None;
static SDL_GLContext gSDLContext = NULL;
static int contextWidth = 0;
static int contextHeight = 0;
static bool contextWasUpdated = false;
static SDL_Window* gSDLWindow = NULL;
static int32_t gViewport[4] = {0, 0, 0, 0}; // {x, y, width, height}
static int gDisplayOrientation = 1; // range [0-3]. 1=landscape.
static float gDisplayDPI = 72.0f;
#if HAVE_GLES2
static GLint uniforms[UNIFORM_COUNT] = {0};
static GLuint program = 0;
#endif // HAVE_GLES2

// Main state.
static struct timeval gStartTime;

// Corner finder results copy, for display to user.
static ARGL_CONTEXT_SETTINGS_REF gArglSettingsCornerFinderImage = NULL;

// ============================================================================
//	Function prototypes
// ============================================================================

static void quit(int rc);
static void reshape(int w, int h);
static void drawView(void);

//static void          init(int argc, char *argv[]);
//static void          usage(char *com);
static void saveParam(const ARParam *param, ARdouble err_min, ARdouble err_avg, ARdouble err_max, void *userdata);


static PFNGLACTIVETEXTUREARBPROC glActiveTexture1 = NULL; //(PFNGLACTIVETEXTUREARBPROC)wglGetProcAddress("glActiveTextureARB");
static PFNGLCLIENTACTIVETEXTUREARBPROC glClientActiveTexture1 = NULL;// (PFNGLCLIENTACTIVETEXTUREARBPROC)wglGetProcAddress("glClientActiveTextureARB");

static void startVideo(void)
{
    char buf[256];
    //snprintf(buf, sizeof(buf), "%s %s", (gPreferenceCameraOpenToken ? gPreferenceCameraOpenToken : ""), (gPreferenceCameraResolutionToken ? gPreferenceCameraResolutionToken : ""));
//	snprintf(buf, sizeof(buf), "-module=LeapMotion -no-deallocate");
	snprintf(buf, sizeof(buf), "-module=LeapMotion -no-deallocate -stereo_part=right");

    vs = new ARVideoSource;
    if (!vs) {
        ARLOGe("Error: Unable to create video source.\n");
        quit(-1);
    } else {
        vs->configure(buf, true, NULL, NULL, 0);
        if (!vs->open()) {
            ARLOGe("Error: Unable to open video source.\n");
            ARLOGi((const char *)"Welcome to artoolkitX Camera Calibrator\n(c)2018 Realmax, Inc. & (c)2017 DAQRI LLC.\n\nUnable to open video source.\n\nPress 'p' for settings and help.");
        }
    }
    gPostVideoSetupDone = false;
}

static void stopVideo(void)
{
    // Stop calibration flow.
    flowStopAndFinal();
    
    if (gCalibration) {
        delete gCalibration;
        gCalibration = nullptr;
    }
    
    if (gArglSettingsCornerFinderImage) {
        arglCleanup(gArglSettingsCornerFinderImage); // Clean up any left-over ARGL data.
        gArglSettingsCornerFinderImage = NULL;
    }
    
    delete vv;
    vv = nullptr;
    delete vs;
    vs = nullptr;
}

static void rereadPreferences(void)
{
    // Re-read preferences.
    gCalibrationSave = getPreferenceCalibrationSave(gPreferences);
    char *csd = getPreferenceCalibSaveDir(gPreferences);
    if (csd && gCalibrationSaveDir && strcmp(gCalibrationSaveDir, csd) == 0) {
        free(csd);
    } else {
        free(gCalibrationSaveDir);
        gCalibrationSaveDir = csd;
    }
    char *csuu = getPreferenceCalibrationServerUploadURL(gPreferences);
    if (csuu && gCalibrationServerUploadURL && strcmp(gCalibrationServerUploadURL, csuu) == 0) {
        free(csuu);
    } else {
        free(gCalibrationServerUploadURL);
        gCalibrationServerUploadURL = csuu;
        fileUploaderFinal(&fileUploadHandle);
        if (csuu) {
            fileUploadHandle = fileUploaderInit(gFileUploadQueuePath, QUEUE_INDEX_FILE_EXTENSION, gCalibrationServerUploadURL, UPLOAD_STATUS_HIDE_AFTER_SECONDS);
            if (!fileUploadHandle) {
                ARLOGe("Error: Could not initialise fileUploadHandle.\n");
            }
        }
    }
    char *csat = getPreferenceCalibrationServerAuthenticationToken(gPreferences);
    if (csat && gCalibrationServerAuthenticationToken && strcmp(gCalibrationServerAuthenticationToken, csat) == 0) {
        free(csat);
    } else {
        free(gCalibrationServerAuthenticationToken);
        gCalibrationServerAuthenticationToken = csat;
    }
    bool changedCameraSettings = false;
    char *crt = getPreferenceCameraResolutionToken(gPreferences);
    if (crt && gPreferenceCameraResolutionToken && strcmp(gPreferenceCameraResolutionToken, crt) == 0) {
        free(crt);
    } else {
        free(gPreferenceCameraResolutionToken);
        gPreferenceCameraResolutionToken = crt;
        changedCameraSettings = true;
    }
    char *cot = getPreferenceCameraOpenToken(gPreferences);
    if (cot && gPreferenceCameraOpenToken && strcmp(gPreferenceCameraOpenToken, cot) == 0) {
        free(cot);
    } else {
        free(gPreferenceCameraOpenToken);
        gPreferenceCameraOpenToken = cot;
        changedCameraSettings = true;
    }
    Calibration::CalibrationPatternType patternType = getPreferencesCalibrationPatternType(gPreferences);
    cv::Size patternSize = getPreferencesCalibrationPatternSize(gPreferences);
    float patternSpacing = getPreferencesCalibrationPatternSpacing(gPreferences);
    if (patternType != gCalibrationPatternType || patternSize != gCalibrationPatternSize || patternSpacing != gCalibrationPatternSpacing) {
        gCalibrationPatternType = patternType;
        gCalibrationPatternSize = patternSize;
        gCalibrationPatternSpacing = patternSpacing;
        changedCameraSettings = true;
    }
    
    if (changedCameraSettings) {
        // Changing camera settings requires complete cancelation of calibration flow,
        // closing of video source, and re-init.
        stopVideo();
        startVideo();
    }
}

int main(int argc, char *argv[])
{
#ifdef DEBUG
    arLogLevel = AR_LOG_LEVEL_DEBUG;
#endif

    // Initialize SDL.
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        ARLOGe("Error: SDL initialisation failed. SDL error: '%s'.\n", SDL_GetError());
        return -1;
    }
    
    // Preferences.
    gPreferences = initPreferences();
    gPreferenceCameraOpenToken = getPreferenceCameraOpenToken(gPreferences);
    gPreferenceCameraResolutionToken = getPreferenceCameraResolutionToken(gPreferences);
    gCalibrationSave = getPreferenceCalibrationSave(gPreferences);
    gCalibrationSaveDir = getPreferenceCalibSaveDir(gPreferences);
    gCalibrationServerUploadURL = getPreferenceCalibrationServerUploadURL(gPreferences);
    gCalibrationServerAuthenticationToken = getPreferenceCalibrationServerAuthenticationToken(gPreferences);
    gCalibrationPatternType = getPreferencesCalibrationPatternType(gPreferences);
    gCalibrationPatternSize = getPreferencesCalibrationPatternSize(gPreferences);
    gCalibrationPatternSpacing = getPreferencesCalibrationPatternSpacing(gPreferences);
    
    gSDLEventPreferencesChanged = SDL_RegisterEvents(1);
    
    // Create a window.
    gSDLWindow = SDL_CreateWindow("artoolkitX Camera Calibration Utility",
                                  SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED,
                                  1280, 720,
                                  SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI
                                  );
    if (!gSDLWindow) {
        ARLOGe("Error creating window: %s.\n", SDL_GetError());
        quit(-1);
    }
    
    // Create an OpenGL context to draw into. If OpenGL 3.2 not available, attempt to fall back to OpenGL 1.5, then OpenGL ES 2.0
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 16);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1); // This is the default.
    SDL_GL_SetSwapInterval(1);
#if HAVE_GL3
//    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
//    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
//    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
//    gSDLContext = SDL_GL_CreateContext(gSDLWindow);
//    if (gSDLContext) {
//        drawAPI = ARG_API_GL3;
//        ARLOGi("Created OpenGL 3.2+ context.\n");
//    } else {
//        ARLOGi("Unable to create OpenGL 3.2 context: %s. Will try OpenGL 1.5.\n", SDL_GetError());
#endif // HAVE_GL3
#if HAVE_GL
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 1);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 5);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, 0);
        gSDLContext = SDL_GL_CreateContext(gSDLWindow);
        if (gSDLContext) {
            drawAPI = ARG_API_GL;
            ARLOGi("Created OpenGL 1.5+ context.\n");
        } else {
            ARLOGi("Unable to create OpenGL 1.5 context: %s. Will try OpenGL ES 2.0\n", SDL_GetError());
#endif // HAVE_GL
#if HAVE_GLES2
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
            gSDLContext = SDL_GL_CreateContext(gSDLWindow);
            if (gSDLContext) {
                drawAPI = ARG_API_GLES2;
                ARLOGi("Created OpenGL ES 2.0+ context.\n");
            } else {
                ARLOGi("Unable to create OpenGL ES 2.0 context: %s.\n", SDL_GetError());
            }
#endif // HAVE_GLES2
#if HAVE_GL
        }
#endif
#if HAVE_GL3
//    }
#endif
    if (drawAPI == ARG_API_None) {
        ARLOGe("No OpenGL context available. Giving up.\n", SDL_GetError());
        quit(-1);
    }

	glActiveTexture1 = (PFNGLACTIVETEXTUREARBPROC)wglGetProcAddress("glActiveTextureARB");
	if (glActiveTexture1 == NULL)
		printf("broken glActiveTexture\n");
	glClientActiveTexture1 = (PFNGLCLIENTACTIVETEXTUREARBPROC)wglGetProcAddress("glClientActiveTextureARB");
	if (glClientActiveTexture1 == NULL)
		printf("broken glClientActiveTexture\n");

    int w, h;
    SDL_GL_GetDrawableSize(SDL_GL_GetCurrentWindow(), &w, &h);
    reshape(w, h);
    
    asprintf(&gFileUploadQueuePath, "%s\\%s", arUtilGetResourcesDirectoryPath(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_USE_APP_CACHE_DIR), QUEUE_DIR);
    // Check for QUEUE_DIR and create if not already existing.
    if (!fileUploaderCreateQueueDir(gFileUploadQueuePath)) {
        ARLOGe("Error: Could not create queue directory.\n");
        exit(-1);
    }
    
    if (gCalibrationServerUploadURL) {
        fileUploadHandle = fileUploaderInit(gFileUploadQueuePath, QUEUE_INDEX_FILE_EXTENSION, gCalibrationServerUploadURL, UPLOAD_STATUS_HIDE_AFTER_SECONDS);
        if (!fileUploadHandle) {
            ARLOGe("Error: Could not initialise fileUploadHandle.\n");
        }
        fileUploaderTickle(fileUploadHandle);
    }
    
    // Calibration prefs.
    ARLOGi("Calbration pattern size X = %d\n", gCalibrationPatternSize.width);
    ARLOGi("Calbration pattern size Y = %d\n", gCalibrationPatternSize.height);
    ARLOGi("Calbration pattern spacing = %f\n", gCalibrationPatternSpacing);
    ARLOGi("Calibration image count maximum = %d\n", gPreferencesCalibImageCountMax);
    
    // Library setup.
    int contextsActiveCount = 1;
    EdenMessageInit(contextsActiveCount);
    EdenGLFontInit(contextsActiveCount);
    EdenGLFontSetFont(EDEN_GL_FONT_ID_Stroke_Roman);
    EdenGLFontSetupFontForContext(0, EDEN_GL_FONT_ID_Stroke_Roman);
    EdenGLFontSetSize(FONT_SIZE);
    
    // Get start time.
    gettimeofday(&gStartTime, NULL);
    
#if HAVE_GLES2
    if (drawAPI == ARG_API_GLES2 && !program) {
        GLuint vertShader = 0, fragShader = 0;
        // A simple shader pair which accepts just a vertex position. Fixed color, no lighting.
        const char vertShaderString[] =
        "attribute vec4 position;\n"
        "uniform vec4 color;\n"
        "uniform mat4 modelViewProjectionMatrix;\n"
        
        "varying vec4 colorVarying;\n"
        "void main()\n"
        "{\n"
        "gl_Position = modelViewProjectionMatrix * position;\n"
        "colorVarying = color;\n"
        "}\n";
        const char fragShaderString[] =
        "#ifdef GL_ES\n"
        "precision mediump float;\n"
        "#endif\n"
        "varying vec4 colorVarying;\n"
        "void main()\n"
        "{\n"
        "gl_FragColor = colorVarying;\n"
        "}\n";
        
        if (program) arglGLDestroyShaders(0, 0, program);
        program = glCreateProgram();
        if (!program) {
            ARLOGe("draw: Error creating shader program.\n");
            quit(-1);
        }
        
        if (!arglGLCompileShaderFromString(&vertShader, GL_VERTEX_SHADER, vertShaderString)) {
            ARLOGe("draw: Error compiling vertex shader.\n");
            arglGLDestroyShaders(vertShader, fragShader, program);
            program = 0;
            quit(-1);
        }
        if (!arglGLCompileShaderFromString(&fragShader, GL_FRAGMENT_SHADER, fragShaderString)) {
            ARLOGe("draw: Error compiling fragment shader.\n");
            arglGLDestroyShaders(vertShader, fragShader, program);
            program = 0;
            quit(-1);
        }
        glAttachShader(program, vertShader);
        glAttachShader(program, fragShader);
        
        glBindAttribLocation(program, ATTRIBUTE_VERTEX, "position");
        if (!arglGLLinkProgram(program)) {
            ARLOGe("draw: Error linking shader program.\n");
            arglGLDestroyShaders(vertShader, fragShader, program);
            program = 0;
            quit(-1);
        }
        arglGLDestroyShaders(vertShader, fragShader, 0); // After linking, shader objects can be deleted.
        
        // Retrieve linked uniform locations.
        uniforms[UNIFORM_MODELVIEW_PROJECTION_MATRIX] = glGetUniformLocation(program, "modelViewProjectionMatrix");
        uniforms[UNIFORM_COLOR] = glGetUniformLocation(program, "color");
    }
#endif // HAVE_GLES2

    startVideo();
    
    // Main loop.
    bool done = false;
    while (!done) {
        
        SDL_Event ev;
        while (SDL_PollEvent(&ev)) {
            if (ev.type == SDL_QUIT /*|| (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE)*/) {
                done = true;
                break;
            } else if (ev.type == SDL_WINDOWEVENT) {
                //ARLOGd("Window event %d.\n", ev.window.event);
                if (ev.window.event == SDL_WINDOWEVENT_RESIZED && ev.window.windowID == SDL_GetWindowID(gSDLWindow)) {
                    //int32_t w = ev.window.data1;
                    //int32_t h = ev.window.data2;
                    int w, h;
                    SDL_GL_GetDrawableSize(gSDLWindow, &w, &h);
                    reshape(w, h);
                }
            } else if (ev.type == SDL_KEYDOWN) {
                if (EdenMessageKeyboardRequired()) {
                    EdenMessageInputKeyboard(ev.key.keysym.sym);
                } else if (ev.key.keysym.sym == SDLK_ESCAPE) {
                    flowHandleEvent(EVENT_BACK_BUTTON);
                } else if (ev.key.keysym.sym == SDLK_SPACE) {
                    flowHandleEvent(EVENT_TOUCH);
                } else if ((ev.key.keysym.sym == SDLK_COMMA && (ev.key.keysym.mod & KMOD_LGUI)) || ev.key.keysym.sym == SDLK_p) {
                    showPreferences(gPreferences);
                }
            } else if (gSDLEventPreferencesChanged != 0 && ev.type == gSDLEventPreferencesChanged) {
                rereadPreferences();
            }
        }
        
        if (vs->isOpen()) {
			//printf("Get frame.\n");
            if (vs->captureFrame()) {
                gFrameCount++; // Increment ARToolKit FPS counter.
#ifdef DEBUG
                if (gFrameCount % 150 == 0) {
                    ARLOGi("*** Camera - %f (frame/sec)\n", (double)gFrameCount/arUtilTimer());
                    gFrameCount = 0;
                    arUtilTimerReset();
                }
#endif
                if (!gPostVideoSetupDone) {
                    
                    gCameraIsFrontFacing = false;
                    AR2VideoParamT *vid = vs->getAR2VideoParam();
                    
                    if (vid->module == AR_VIDEO_MODULE_AVFOUNDATION) {
                        int frontCamera;
                        if (ar2VideoGetParami(vid, AR_VIDEO_PARAM_AVFOUNDATION_CAMERA_POSITION, &frontCamera) >= 0) {
                            gCameraIsFrontFacing = (frontCamera == AR_VIDEO_AVFOUNDATION_CAMERA_POSITION_FRONT);
                        }
                    }
                    bool contentRotate90, contentFlipV, contentFlipH;
                    if (gDisplayOrientation == 1) { // Landscape with top of device at left.
                        contentRotate90 = false;
                        contentFlipV = gCameraIsFrontFacing;
                        contentFlipH = gCameraIsFrontFacing;
                    } else if (gDisplayOrientation == 2) { // Portrait upside-down.
                        contentRotate90 = true;
                        contentFlipV = !gCameraIsFrontFacing;
                        contentFlipH = true;
                    } else if (gDisplayOrientation == 3) { // Landscape with top of device at right.
                        contentRotate90 = false;
                        contentFlipV = !gCameraIsFrontFacing;
                        contentFlipH = (!gCameraIsFrontFacing);
                    } else /*(gDisplayOrientation == 0)*/ { // Portait
                        contentRotate90 = true;
                        contentFlipV = gCameraIsFrontFacing;
                        contentFlipH = false;
                    }
                    
                    // Setup a route for rendering the colour background image.
                    vv = new ARVideoView;
                    if (!vv) {
                        ARLOGe("Error: unable to create video view.\n");
                        quit(-1);
                    }
                    vv->setRotate90(contentRotate90);
                    vv->setFlipH(contentFlipH);
                    vv->setFlipV(contentFlipV);
                    vv->setScalingMode(ARVideoView::ScalingMode::SCALE_MODE_FIT);
                    vv->initWithVideoSource(*vs, contextWidth, contextHeight);
                    ARLOGi("Content %dx%d (wxh) will display in GL context %dx%d%s.\n", vs->getVideoWidth(), vs->getVideoHeight(), contextWidth, contextHeight, (contentRotate90 ? " rotated" : ""));
                    vv->getViewport(gViewport);
                    
                    // Setup a route for rendering the mono background image.
                    ARParam idealParam;
                    arParamClear(&idealParam, vs->getVideoWidth(), vs->getVideoHeight(), AR_DIST_FUNCTION_VERSION_DEFAULT);
                    if ((gArglSettingsCornerFinderImage = arglSetupForCurrentContext(&idealParam, AR_PIXEL_FORMAT_MONO)) == NULL) {
                        ARLOGe("Unable to setup argl.\n");
                        quit(-1);
                    }
                    if (!arglDistortionCompensationSet(gArglSettingsCornerFinderImage, FALSE)) {
                        ARLOGe("Unable to setup argl.\n");
                        quit(-1);
                    }
                    arglSetRotate90(gArglSettingsCornerFinderImage, contentRotate90);
                    arglSetFlipV(gArglSettingsCornerFinderImage, contentFlipV);
                    arglSetFlipH(gArglSettingsCornerFinderImage, contentFlipH);
                    
                    //
                    // Calibration init.
                    //
                    
                    gCalibration = new Calibration(gCalibrationPatternType, gPreferencesCalibImageCountMax, gCalibrationPatternSize, gCalibrationPatternSpacing, vs->getVideoWidth(), vs->getVideoHeight());
                    if (!gCalibration) {
                        ARLOGe("Error initialising calibration.\n");
                        quit(-1);
                    }
                    
                    if (!flowInitAndStart(gCalibration, saveParam, NULL)) {
                        ARLOGe("Error: Could not initialise and start flow.\n");
                        quit(-1);
                    }
                    
                    // For FPS statistics.
                    arUtilTimerReset();
                    gFrameCount = 0;
                    
                    gPostVideoSetupDone = true;
                } // !gPostVideoSetupDone
                
                if (contextWasUpdated) {
                    vv->setContextSize({contextWidth, contextHeight});
                    vv->getViewport(gViewport);
                }
                
                FLOW_STATE state = flowStateGet();
                if (state == FLOW_STATE_WELCOME || state == FLOW_STATE_DONE || state == FLOW_STATE_CALIBRATING) {
                    
                    // Upload the frame to OpenGL.
                    // Now done as part of the draw call.
                    
                } else if (state == FLOW_STATE_CAPTURING) {
                    
                    gCalibration->frame(vs);

                }
                
            }
            
        } // vs->isOpen()
        
        // The display has changed.
        drawView();
                
        arUtilSleep(1); // 1 millisecond.
    }
    
    stopVideo();
    
    quit(0);
}

void reshape(int w, int h)
{
    contextWidth = w;
    contextHeight = h;
    ARLOGd("Resized to %dx%d.\n", w, h);
    contextWasUpdated = true;
}

static void quit(int rc)
{
    fileUploaderFinal(&fileUploadHandle);
    
    SDL_Quit();
    
    free(gPreferenceCameraOpenToken);
    free(gPreferenceCameraResolutionToken);
    free(gCalibrationServerUploadURL);
    free(gCalibrationServerAuthenticationToken);
    preferencesFinal(&gPreferences);
    
    exit(rc);
}

static void usage(char *com)
{
    ARPRINT("Usage: %s [options]\n", com);
    ARPRINT("Options:\n");
    ARPRINT("  --vconf <video parameter for the camera>\n");
    ARPRINT("  -cornerx=n: specify the number of corners on chessboard in X direction.\n");
    ARPRINT("  -cornery=n: specify the number of corners on chessboard in Y direction.\n");
    ARPRINT("  -imagenum=n: specify the number of images captured for calibration.\n");
    ARPRINT("  -pattwidth=n: specify the square width in the chessbaord.\n");
    ARPRINT("  -h -help --help: show this message\n");
    exit(0);
}

/*
static void init(int argc, char *argv[])
{
    ARGViewport     viewport;
    char           *vconf = NULL;
    int             i;
    int             gotTwoPartOption;
    int             screenWidth, screenHeight, screenMargin;
    
    chessboardCornerNumX = 0;
    chessboardCornerNumY = 0;
    calibImageNum        = 0;
    patternWidth         = 0.0f;
    
    arMalloc(cwd, char, MAXPATHLEN);
    if (!getcwd(cwd, MAXPATHLEN)) ARLOGe("Unable to read current working directory.\n");
    else ARPRINT("Current working directory is '%s'\n", cwd);
    
    i = 1; // argv[0] is name of app, so start at 1.
    while (i < argc) {
        gotTwoPartOption = FALSE;
        // Look for two-part options first.
        if ((i + 1) < argc) {
            if (strcmp(argv[i], "--vconf") == 0) {
                i++;
                vconf = argv[i];
                gotTwoPartOption = TRUE;
            }
        }
        if (!gotTwoPartOption) {
            // Look for single-part options.
            if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "-h") == 0) {
                usage(argv[0]);
            } else if (strcmp(argv[i], "--version") == 0 || strcmp(argv[i], "-version") == 0 || strcmp(argv[i], "-v") == 0) {
                ARPRINT("%s version %s\n", argv[0], AR_HEADER_VERSION_STRING);
                exit(0);
            } else if( strncmp(argv[i], "-cornerx=", 9) == 0 ) {
                if( sscanf(&(argv[i][9]), "%d", &chessboardCornerNumX) != 1 ) usage(argv[0]);
                if( chessboardCornerNumX <= 0 ) usage(argv[0]);
            } else if( strncmp(argv[i], "-cornery=", 9) == 0 ) {
                if( sscanf(&(argv[i][9]), "%d", &chessboardCornerNumY) != 1 ) usage(argv[0]);
                if( chessboardCornerNumY <= 0 ) usage(argv[0]);
            } else if( strncmp(argv[i], "-imagenum=", 10) == 0 ) {
                if( sscanf(&(argv[i][10]), "%d", &calibImageNum) != 1 ) usage(argv[0]);
                if( calibImageNum <= 0 ) usage(argv[0]);
            } else if( strncmp(argv[i], "-pattwidth=", 11) == 0 ) {
                if( sscanf(&(argv[i][11]), "%f", &patternWidth) != 1 ) usage(argv[0]);
                if( patternWidth <= 0 ) usage(argv[0]);
            } else {
                ARLOGe("Error: invalid command line argument '%s'.\n", argv[i]);
                usage(argv[0]);
            }
        }
        i++;
    }
    if( chessboardCornerNumX == 0 ) chessboardCornerNumX = CHESSBOARD_CORNER_NUM_X;
    if( chessboardCornerNumY == 0 ) chessboardCornerNumY = CHESSBOARD_CORNER_NUM_Y;
    if( calibImageNum == 0 )        calibImageNum = CALIB_IMAGE_NUM;
    if( patternWidth == 0.0f )       patternWidth = (float)CHESSBOARD_PATTERN_WIDTH;
    ARPRINT("CHESSBOARD_CORNER_NUM_X = %d\n", chessboardCornerNumX);
    ARPRINT("CHESSBOARD_CORNER_NUM_Y = %d\n", chessboardCornerNumY);
    ARPRINT("CHESSBOARD_PATTERN_WIDTH = %f\n", patternWidth);
    ARPRINT("CALIB_IMAGE_NUM = %d\n", calibImageNum);
    ARPRINT("Video parameter: %s\n", vconf);
    
*/

static void drawBackground(const float width, const float height, const float x, const float y, const bool drawBorder)
{
    GLfloat vertices[4][2];
#if HAVE_GLES2
    GLfloat colorBlack50[4] = {0.0f, 0.0f, 0.0f, 0.5f}; // 50% transparent black.
    GLfloat colorWhite[4] = {1.0f, 1.0f, 1.0f, 1.0f}; // Opaque white.
#endif // HAVE_GLES2

    vertices[0][0] = x; vertices[0][1] = y;
    vertices[1][0] = width + x; vertices[1][1] = y;
    vertices[2][0] = width + x; vertices[2][1] = height + y;
    vertices[3][0] = x; vertices[3][1] = height + y;
    
#if !HAVE_GLES2
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glVertexPointer(2, GL_FLOAT, 0, vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glClientActiveTexture1(GL_TEXTURE0);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glColor4f(0.0f, 0.0f, 0.0f, 0.5f);    // 50% transparent black.
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    if (drawBorder) {
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f); // Opaque white.
        glLineWidth(1.0f);
        glDrawArrays(GL_LINE_LOOP, 0, 4);
    }
#else
    glStateCacheDisableDepthTest();
    glStateCacheBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glStateCacheEnableBlend();
    
    glVertexAttribPointer(ATTRIBUTE_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(ATTRIBUTE_VERTEX);
    glUniform4fv(uniforms[UNIFORM_COLOR], 1, colorBlack50);
    glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
    if (drawBorder) {
        glUniform4fv(uniforms[UNIFORM_COLOR], 1, colorWhite);
        glLineWidth(1.0f);
        glDrawArrays(GL_LINE_LOOP, 0, 4);
    }
#endif // !HAVE_GLES2
}

// An animation while we're waiting.
// Designed to be drawn on background of at least 3xsquareSize wide and tall.
static void drawBusyIndicator(int positionX, int positionY, int squareSize, struct timeval *tp)
{
#if !HAVE_GLES2
    const GLfloat square_vertices [4][2] = { {0.5f, 0.5f}, {squareSize - 0.5f, 0.5f}, {squareSize - 0.5f, squareSize - 0.5f}, {0.5f, squareSize - 0.5f} };
    int i;
    
    int hundredthSeconds = (int)tp->tv_usec / 1E4;
    
    // Set up drawing.
    glPushMatrix();
    glLoadIdentity();
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);
    glDisable(GL_TEXTURE_2D);
    glDisable(GL_BLEND);
    glVertexPointer(2, GL_FLOAT, 0, square_vertices);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_NORMAL_ARRAY);
    glClientActiveTexture1(GL_TEXTURE0);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    
    for (i = 0; i < 4; i++) {
        glLoadIdentity();
        glTranslatef((float)(positionX + ((i + 1)/2 != 1 ? -squareSize : 0.0f)), (float)(positionY + (i / 2 == 0 ? 0.0f : -squareSize)), 0.0f); // Order: UL, UR, LR, LL.
        if (i == hundredthSeconds / 25) {
            unsigned char r, g, b;
            int secDiv255 = (int)tp->tv_usec / 3921;
            int secMod6 = tp->tv_sec % 6;
            if (secMod6 == 0) {
                r = 255; g = secDiv255; b = 0;
            } else if (secMod6 == 1) {
                r = secDiv255; g = 255; b = 0;
            } else if (secMod6 == 2) {
                r = 0; g = 255; b = secDiv255;
            } else if (secMod6 == 3) {
                r = 0; g = secDiv255; b = 255;
            } else if (secMod6 == 4) {
                r = secDiv255; g = 0; b = 255;
            } else {
                r = 255; g = 0; b = secDiv255;
            }
            glColor4ub(r, g, b, 255);
            glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
        }
        glColor4ub(255, 255, 255, 255);
        glDrawArrays(GL_LINE_LOOP, 0, 4);
    }
    
    glPopMatrix();
#endif // !HAVE_GLES2
}

void drawView(void)
{
    GLfloat p[16], m[16];
    int i;
    struct timeval time;
    float left, right, bottom, top;
    GLfloat *vertices = NULL;
    GLint vertexCount;

    // Get frame time.
    gettimeofday(&time, NULL);
    
    SDL_GL_MakeCurrent(gSDLWindow, gSDLContext);
    
    // Clean the OpenGL context.
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    //
    // Setup for drawing video frame.
    //
    glViewport(gViewport[0], gViewport[1], gViewport[2], gViewport[3]);
    
    FLOW_STATE state = flowStateGet();
    if (state == FLOW_STATE_WELCOME || state == FLOW_STATE_DONE || state == FLOW_STATE_CALIBRATING) {
        
        // Display the current frame
        vv->draw(vs);
        
    } else if (state == FLOW_STATE_CAPTURING) {
        
        // Grab a lock while we're using the data to prevent it being changed underneath us.
        int cornerFoundAllFlag;
        std::vector<cv::Point2f> corners;
        ARUint8 *videoFrame;
        gCalibration->cornerFinderResultsLockAndFetch(&cornerFoundAllFlag, corners, &videoFrame);
        
        // Display the current frame.
        if (videoFrame) arglPixelBufferDataUpload(gArglSettingsCornerFinderImage, videoFrame);
        arglDispImage(gArglSettingsCornerFinderImage, NULL);
        
        //
        // Setup for drawing on top of video frame, in video pixel coordinates.
        //
        mtxLoadIdentityf(p);
        if (vv->rotate90()) mtxRotatef(p, 90.0f, 0.0f, 0.0f, -1.0f);
        if (vv->flipV()) {
            bottom = (float)vs->getVideoHeight();
            top = 0.0f;
        } else {
            bottom = 0.0f;
            top = (float)vs->getVideoHeight();
        }
        if (vv->flipH()) {
            left = (float)vs->getVideoWidth();
            right = 0.0f;
        } else {
            left = 0.0f;
            right = (float)vs->getVideoWidth();
        }
        mtxOrthof(p, left, right, bottom, top, -1.0f, 1.0f);
        mtxLoadIdentityf(m);
#if !HAVE_GLES2
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(p);
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(m);
        glDisable(GL_DEPTH_TEST);
        glDisable(GL_LIGHTING);
        glDisable(GL_BLEND);
        glActiveTexture1(GL_TEXTURE0);
        glDisable(GL_TEXTURE_2D);
#else
        glStateCacheDisableDepthTest();
        glStateCacheDisableBlend();
#endif // !HAVE_GLES2

        // Draw the crosses marking the corner positions.
        const float colorRed[4] = {1.0f, 0.0f, 0.0f, 1.0f};
        const float colorGreen[4] = {0.0f, 1.0f, 0.0f, 1.0f};
        vertexCount = (GLint)corners.size()*4;
        if (vertexCount > 0) {
            float fontSizeScaled = FONT_SIZE * (float)vs->getVideoHeight()/(float)(gViewport[(gDisplayOrientation % 2) == 1 ? 3 : 2]);
#if !HAVE_GLES2
            glColor4fv(cornerFoundAllFlag ? colorRed : colorGreen);
#endif // !HAVE_GLES2
            EdenGLFontSetSize(fontSizeScaled);
            EdenGLFontSetColor(cornerFoundAllFlag ? colorRed : colorGreen);
            arMalloc(vertices, GLfloat, vertexCount*2); // 2 coords per vertex.
            for (i = 0; i < corners.size(); i++) {
                vertices[i*8    ] = corners[i].x - 5.0f;
                vertices[i*8 + 1] = vs->getVideoHeight() - corners[i].y - 5.0f;
                vertices[i*8 + 2] = corners[i].x + 5.0f;
                vertices[i*8 + 3] = vs->getVideoHeight() - corners[i].y + 5.0f;
                vertices[i*8 + 4] = corners[i].x - 5.0f;
                vertices[i*8 + 5] = vs->getVideoHeight() - corners[i].y + 5.0f;
                vertices[i*8 + 6] = corners[i].x + 5.0f;
                vertices[i*8 + 7] = vs->getVideoHeight() - corners[i].y - 5.0f;
                
                unsigned char buf[12]; // 10 digits in INT32_MAX, plus sign, plus null.
                sprintf((char *)buf, "%d\n", i);
                
                GLfloat mvp[16];
#if !HAVE_GLES2
                glPushMatrix();
                glLoadIdentity();
                glTranslatef(corners[i].x, vs->getVideoHeight() - corners[i].y, 0.0f);
                glRotatef((float)(gDisplayOrientation - 1) * -90.0f, 0.0f, 0.0f, 1.0f); // Orient the text to the user.
#else
                mtxLoadMatrixf(mvp, p);
                mtxMultMatrixf(mvp, m);
                mtxTranslatef(mvp, corners[i].x, vs->getVideoHeight() - corners[i].y, 0.0f);
                mtxRotatef(mvp, (float)(gDisplayOrientation - 1) * -90.0f, 0.0f, 0.0f, 1.0f); // Orient the text to the user.

#endif // !HAVE_GLES2
                EdenGLFontDrawLine(0, mvp, buf, 0.0f, 0.0f, H_OFFSET_VIEW_LEFT_EDGE_TO_TEXT_LEFT_EDGE, V_OFFSET_VIEW_BOTTOM_TO_TEXT_BASELINE); // These alignment modes don't require setting of EdenGLFontSetViewSize().
#if !HAVE_GLES2
                glPopMatrix();
#endif // !HAVE_GLES2
            }
            EdenGLFontSetSize(FONT_SIZE);
            float colorWhite[4] = {1.0f, 1.0f, 1.0f, 1.0f};;
            EdenGLFontSetColor(colorWhite);
        }
        
        gCalibration->cornerFinderResultsUnlock();
        
        if (vertexCount > 0) {
#if !HAVE_GLES2
            glVertexPointer(2, GL_FLOAT, 0, vertices);
            glEnableClientState(GL_VERTEX_ARRAY);
            glDisableClientState(GL_NORMAL_ARRAY);
            glClientActiveTexture1(GL_TEXTURE0);
            glDisableClientState(GL_TEXTURE_COORD_ARRAY);
#else
            glUseProgram(program);
            GLfloat mvp[16];
            mtxLoadMatrixf(mvp, p);
            mtxMultMatrixf(mvp, m);
            glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEW_PROJECTION_MATRIX], 1, GL_FALSE, mvp);
            glUniform4fv(uniforms[UNIFORM_COLOR], 1, cornerFoundAllFlag ? colorRed : colorGreen);
            
            glVertexAttribPointer(ATTRIBUTE_VERTEX, 2, GL_FLOAT, GL_FALSE, 0, vertices);
            glEnableVertexAttribArray(ATTRIBUTE_VERTEX);
#endif // !HAVE_GLES2
            glLineWidth(2.0f);
            glDrawArrays(GL_LINES, 0, vertexCount);
            free(vertices);
        }
    }
    
    //
    // Setup for drawing on top of video frame, in viewPort coordinates.
    //
#if 0
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    bottom = 0.0f;
    top = (float)(viewPort[viewPortIndexHeight]);
    left = 0.0f;
    right = (float)(viewPort[viewPortIndexWidth]);
    glOrthof(left, right, bottom, top, -1.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    EdenGLFontSetViewSize(right, top);
    EdenMessageSetViewSize(right, top, gDisplayDPI);
#endif
    
    //
    // Setup for drawing on screen, with correct orientation for user.
    //
    glViewport(0, 0, contextWidth, contextHeight);
    bottom = 0.0f;
    top = (float)contextHeight;
    left = 0.0f;
    right = (float)contextWidth;
    mtxLoadIdentityf(p);
    mtxOrthof(p, left, right, bottom, top, -1.0f, 1.0f);
    mtxLoadIdentityf(m);
#if !HAVE_GLES2
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(p);
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(m);
#else
    glUseProgram(program);
    glUniformMatrix4fv(uniforms[UNIFORM_MODELVIEW_PROJECTION_MATRIX], 1, GL_FALSE, p);
#endif // !HAVE_GLES2

    EdenGLFontSetViewSize(right, top);
    EdenMessageSetViewSize(right, top);
    EdenMessageSetBoxParams(600.0f, 20.0f);
    float statusBarHeight = EdenGLFontGetHeight() + 4.0f; // 2 pixels above, 2 below.
    
    // Draw status bar with centred status message.
    if (statusBarMessage[0]) {
        drawBackground(right, statusBarHeight, 0.0f, 0.0f, false);
        glDisable(GL_BLEND);
        EdenGLFontDrawLine(0, p, statusBarMessage, 0.0f, 2.0f, H_OFFSET_VIEW_CENTER_TO_TEXT_CENTER, V_OFFSET_VIEW_BOTTOM_TO_TEXT_BASELINE);
    }
    
    // If background tasks are proceeding, draw a status box.
    if (fileUploadHandle) {
        char uploadStatus[UPLOAD_STATUS_BUFFER_LEN];
        int status = fileUploaderStatusGet(fileUploadHandle, uploadStatus, &time);
        if (status > 0) {
            const int squareSize = (int)(16.0f * (float)gDisplayDPI / 160.f) ;
            float x, y, w, h;
            float textWidth = EdenGLFontGetLineWidth((unsigned char *)uploadStatus);
            w = textWidth + 3*squareSize + 2*4.0f /*text margin*/ + 2*4.0f /* box margin */;
            h = MAX(FONT_SIZE, 3*squareSize) + 2*4.0f /* box margin */;
            x = right - (w + 2.0f);
            y = statusBarHeight + 2.0f;
            drawBackground(w, h, x, y, true);
            if (status == 1) drawBusyIndicator((int)(x + 4.0f + 1.5f*squareSize), (int)(y + 4.0f + 1.5f*squareSize), squareSize, &time);
            EdenGLFontDrawLine(0, p, (unsigned char *)uploadStatus, x + 4.0f + 3*squareSize, y + (h - FONT_SIZE)/2.0f, H_OFFSET_VIEW_LEFT_EDGE_TO_TEXT_LEFT_EDGE, V_OFFSET_VIEW_BOTTOM_TO_TEXT_BASELINE);
        }
    }
    
    // If a message should be onscreen, draw it.
    //if (gEdenMessageDrawRequired) EdenMessageDraw(0, p);
    
    SDL_GL_SwapWindow(gSDLWindow);
}


// Save parameters file and index file with info about it, then signal thread that it's ready for upload.
static void saveParam(const ARParam *param, ARdouble err_min, ARdouble err_avg, ARdouble err_max, void *userdata)
{
    int i;
#define SAVEPARAM_PATHNAME_LEN MAXPATHLEN
    char indexPathname[SAVEPARAM_PATHNAME_LEN];
    char paramPathname[SAVEPARAM_PATHNAME_LEN];
    char indexUploadPathname[SAVEPARAM_PATHNAME_LEN];
    
    // Get the current time. It will be used for file IDs, plus a timestamp for the parameters file.
    time_t ourClock = time(NULL);
    if (ourClock == (time_t)-1) {
        ARLOGe("Error reading time and date.\n");
        return;
    }
    //struct tm *timeptr = localtime(&ourClock);
    struct tm *timeptr = gmtime(&ourClock);
    if (!timeptr) {
        ARLOGe("Error converting time and date to UTC.\n");
        return;
    }
    int ID = timeptr->tm_hour*10000 + timeptr->tm_min*100 + timeptr->tm_sec;
    
    // Save the parameter file.
    snprintf(paramPathname, SAVEPARAM_PATHNAME_LEN, "%s/%s/%06d-camera_para.dat", arUtilGetResourcesDirectoryPath(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_USE_APP_CACHE_DIR), QUEUE_DIR, ID);
	printf("file: %s\n", paramPathname);
    //if (arParamSave(strcat(strcat(docsPath,"/"),paramPathname), 1, param) < 0) {
    if (arParamSave(paramPathname, 1, param) < 0) {
        
        ARLOGe("Error writing camera_para.dat file.\n");
        
    } else {
        
        // Get main device identifier and focal length from video module.
        char *device_id = NULL;
        char *name = NULL;
        char *focal_length = NULL;
        
        AR2VideoParamT *vid = vs->getAR2VideoParam();
        if (ar2VideoGetParams(vid, AR_VIDEO_PARAM_DEVICEID, &device_id) < 0 || !device_id) {
            ARLOGe("Error fetching camera device identification.\n");
        }
        if (ar2VideoGetParams(vid, AR_VIDEO_PARAM_NAME, &name) < 0 || !name) {
            ARLOGe("Error fetching camera name.\n");
        }
        
        if (vid->module == AR_VIDEO_MODULE_AVFOUNDATION) {
            int focalPreset;
            ar2VideoGetParami(vid, AR_VIDEO_PARAM_AVFOUNDATION_FOCUS_PRESET, &focalPreset);
            switch (focalPreset) {
                case AR_VIDEO_AVFOUNDATION_FOCUS_MACRO:
                    focal_length = strdup("0.01");
                    break;
                case AR_VIDEO_AVFOUNDATION_FOCUS_0_3M:
                    focal_length = strdup("0.3");
                    break;
                case AR_VIDEO_AVFOUNDATION_FOCUS_1_0M:
                    focal_length = strdup("1.0");
                    break;
                case AR_VIDEO_AVFOUNDATION_FOCUS_INF:
                    focal_length = strdup("1000000.0");
                    break;
                default:
                    break;
            }
        }
        if (!focal_length) {
            // Not known at present, so just send 0.000.
            focal_length = strdup("0.000");
        }
        
        if (gCalibrationSave || true) {
            
            // Assemble the filename.
            char calibrationSavePathname[SAVEPARAM_PATHNAME_LEN];
            snprintf(calibrationSavePathname, SAVEPARAM_PATHNAME_LEN, "%s/camera_para-", gCalibrationSaveDir);
            size_t len = strlen(calibrationSavePathname);
            int i = 0;
            const char *identifier = (device_id ? device_id : (name ? name : ""));
            while (identifier[i] && (len + i + 2 < SAVEPARAM_PATHNAME_LEN)) {
                calibrationSavePathname[len + i] = (identifier[i] == '/' || identifier[i] == '\\' ? '_' : identifier[i]);
                i++;
            }
            calibrationSavePathname[len + i] = '\0';
            len = strlen(calibrationSavePathname);
            snprintf(&calibrationSavePathname[len], SAVEPARAM_PATHNAME_LEN - len, "-0-%dx%d", vs->getVideoWidth(), vs->getVideoHeight()); // camera_index is always 0 for desktop platforms.
            len = strlen(calibrationSavePathname);
            if (strcmp(focal_length, "0.000") != 0) {
                snprintf(&calibrationSavePathname[len], SAVEPARAM_PATHNAME_LEN - len, "-%s", focal_length);
                len = strlen(calibrationSavePathname);
            }
            snprintf(&calibrationSavePathname[len], SAVEPARAM_PATHNAME_LEN - len, ".dat");

            if (cp_f(paramPathname, calibrationSavePathname) != 0) {
                ARLOGe("Error saving calibration to '%s'", calibrationSavePathname);
                ARLOGperror(NULL);
            } else {
                ARLOGi("Saved calibration to '%s'.\n", calibrationSavePathname);
            }
        }

        // Check for early exit.
        if (!gCalibrationServerUploadURL || !device_id) {
            if (remove(paramPathname) < 0) {
                ARLOGe("Error removing temporary file '%s'.\n", paramPathname);
                ARLOGperror(NULL);
            }
            free(device_id);
            free(focal_length);
            return;
        };

        //
        // Write an upload index file with the data for the server database entry.
        //
        
        // Open the file.
        snprintf(indexPathname, SAVEPARAM_PATHNAME_LEN, "%s/%s/%06d-index", arUtilGetResourcesDirectoryPath(AR_UTIL_RESOURCES_DIRECTORY_BEHAVIOR_USE_APP_CACHE_DIR), QUEUE_DIR, ID);
        FILE *fp;
        bool goodWrite = true;
        if (!(fp = fopen(indexPathname, "wb"))) {
            ARLOGe("Error opening upload index file '%s'.\n", indexPathname);
            goodWrite = false;
        }
        
        // Add a version to the request. "2" if sending a v5 distortion function file, "1" otherwise.
        if (goodWrite) fprintf(fp, "version,%d\n", param->dist_function_version == 5 ? 2 : 1);
        
        // File name.
        if (goodWrite) fprintf(fp, "file,%s\n", paramPathname);
        
        // UTC date and time, in format "1999-12-31 23:59:59 UTC".
        if (goodWrite) {
            char timestamp[26+8] = "";
            if (!strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S +0000", timeptr)) { // Use explicit "+0000" rather than %z because %z is undefined either UTC or local time zone when timestamp is created with gmtime().
                ARLOGe("Error formatting time and date.\n");
                goodWrite = false;
            } else {
                fprintf(fp, "timestamp,%s\n", timestamp);
            }
        }
        
        // OS: name/arch/version.
        if (goodWrite) {
            char *os_name = arUtilGetOSName();
            char *os_arch = arUtilGetCPUName();
            char *os_version = arUtilGetOSVersion();
            fprintf(fp, "os_name,%s\nos_arch,%s\nos_version,%s\n", os_name, os_arch, os_version);
            free(os_name);
            free(os_arch);
            free(os_version);
        }
        
        // Camera identifier.
        if (goodWrite) {
            fprintf(fp, "device_id,%s\n", device_id);
        }
        
        // Focal length in metres.
        if (goodWrite) {
            fprintf(fp, "focal_length,%s\n", focal_length);
        }
        
        // Camera index.
        if (goodWrite) {
            char camera_index[12]; // 10 digits in INT32_MAX, plus sign, plus null.
            snprintf(camera_index, 12, "%d", 0); // Always zero for desktop platforms.
            fprintf(fp, "camera_index,%s\n", camera_index);
        }
        
        // Front or rear facing.
        if (goodWrite) {
            char camera_face[6]; // "front" or "rear", plus null.
            snprintf(camera_face, 6, "%s", (gCameraIsFrontFacing ? "front" : "rear"));
            fprintf(fp, "camera_face,%s\n", camera_face);
        }
        
        // Camera dimensions.
        if (goodWrite) {
            char camera_width[12]; // 10 digits in INT32_MAX, plus sign, plus null.
            char camera_height[12]; // 10 digits in INT32_MAX, plus sign, plus null.
            snprintf(camera_width, 12, "%d", vs->getVideoWidth());
            snprintf(camera_height, 12, "%d", vs->getVideoHeight());
            fprintf(fp, "camera_width,%s\n", camera_width);
            fprintf(fp, "camera_height,%s\n", camera_height);
        }
        
        // Calibration error.
        if (goodWrite) {
            char err_min_ascii[12];
            char err_avg_ascii[12];
            char err_max_ascii[12];
            snprintf(err_min_ascii, 12, "%f", err_min);
            snprintf(err_avg_ascii, 12, "%f", err_avg);
            snprintf(err_max_ascii, 12, "%f", err_max);
            fprintf(fp, "err_min,%s\n", err_min_ascii);
            fprintf(fp, "err_avg,%s\n", err_avg_ascii);
            fprintf(fp, "err_max,%s\n", err_max_ascii);
        }
        
        // IP address will be derived from connect.
        
        // Hash the shared secret.
        if (goodWrite) {
            unsigned char ss_md5[MD5_DIGEST_LENGTH];
            char ss_ascii[MD5_DIGEST_LENGTH*2 + 1]; // space for null terminator.
            if (false) {//!MD5((unsigned char *)gCalibrationServerAuthenticationToken, (MD5_COUNT_t)strlen(gCalibrationServerAuthenticationToken), ss_md5)) {
                ARLOGe("Error calculating md5.\n");
                goodWrite = false;
            } else {
                for (i = 0; i < MD5_DIGEST_LENGTH; i++) snprintf(&(ss_ascii[i*2]), 3, "%.2hhx", ss_md5[i]);
                fprintf(fp, "ss,%s\n", ss_ascii);
            }
        }
        
        // Done writing index file.
        fclose(fp);
        
        if (goodWrite) {
            // Rename the file with QUEUE_INDEX_FILE_EXTENSION file extension so it's picked up in uploader.
            snprintf(indexUploadPathname, SAVEPARAM_PATHNAME_LEN, "%s." QUEUE_INDEX_FILE_EXTENSION, indexPathname);
            if (rename(indexPathname, indexUploadPathname) < 0) {
                ARLOGe("Error renaming temporary file '%s'.\n", indexPathname);
                goodWrite = false;
            } else {
                // Kick off an upload handling cycle.
                fileUploaderTickle(fileUploadHandle);
            }
        }
        
        if (!goodWrite) {
            // If something went wrong, delete the index and param files.
            if (indexPathname[0]) {
                if (remove(indexPathname) < 0) {
                    ARLOGe("Error removing temporary file '%s'.\n", indexPathname);
                    ARLOGperror(NULL);
                }
            }
            if (remove(paramPathname) < 0) {
                ARLOGe("Error removing temporary file '%s'.\n", paramPathname);
                ARLOGperror(NULL);
            }
        }

        free(device_id);
        free(focal_length);
    }
}


