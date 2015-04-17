/**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
 * Copyright (c) 2012 Benjamin Venditti <benjamin.venditti@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 **/


#include <stdio.h>

#include "psmove_tracker.h"

#include "../../../external/iniparser/iniparser.h"

#include "../camera_control.h"
#include "../camera_control_private.h"

void camera_control_backup_system_settings(CameraControl* cc, const char* file) {
    // This is called shortly after the camera is loaded.
    // Its intention is to backup whatever settings previously existed prior to running the psmoveapi.
#if !defined(CAMERA_CONTROL_USE_CL_DRIVER) && defined(PSMOVE_USE_PSEYE)
    // In the case of the PSEYE, we'll just use the PS3EYEDriver defaults.
    int AutoAGC = -1;
    int Gain = -1;
    int AutoAWB = -1;
    int Exposure = -1;
    int Sharpness = -1;
    int Contrast = -1;
    int Brightness = -1;
    //int Hue = -1;
    int wbR = -1;
    int wbB = -1;
    int wbG = -1;
    //int hflip = -1;
    //int vflip = -1;

    AutoAGC = ps3eye_get_parameter(cc->eye, PS3EYE_AUTO_GAIN);
    Gain = ps3eye_get_parameter(cc->eye, PS3EYE_GAIN);
    AutoAWB = ps3eye_get_parameter(cc->eye, PS3EYE_AUTO_WHITEBALANCE);
    Exposure = ps3eye_get_parameter(cc->eye, PS3EYE_EXPOSURE);
    Sharpness = ps3eye_get_parameter(cc->eye, PS3EYE_SHARPNESS);
    Contrast = ps3eye_get_parameter(cc->eye, PS3EYE_CONTRAST);
    Brightness = ps3eye_get_parameter(cc->eye, PS3EYE_BRIGHTNESS);
    //Hue = ps3eye_get_parameter(cc->eye, PS3EYE_HUE);
    wbR = ps3eye_get_parameter(cc->eye, PS3EYE_REDBALANCE);
    wbB = ps3eye_get_parameter(cc->eye, PS3EYE_BLUEBALANCE);
    wbG = ps3eye_get_parameter(cc->eye, PS3EYE_GREENBALANCE);
    //hflip = ps3eye_get_parameter(cc->eye, PS3EYE_HFLIP);
    //vflip = ps3eye_get_parameter(cc->eye, PS3EYE_VFLIP);

    dictionary* ini = dictionary_new(0);
    iniparser_set(ini, "PSEye", 0);
    iniparser_set_int(ini, "PSEye:AutoAGC", AutoAGC);
    iniparser_set_int(ini, "PSEye:Gain", Gain);
    iniparser_set_int(ini, "PSEye:AutoAWB", AutoAWB);
    iniparser_set_int(ini, "PSEye:Exposure", Exposure);
    iniparser_set_int(ini, "PSEye:Sharpness", PS3EYE_SHARPNESS);
    iniparser_set_int(ini, "PSEye:Contrast", Contrast);
    iniparser_set_int(ini, "PSEye:Brightness", Brightness);
    //iniparser_set_int(ini, "PSEye:Hue", Hue);
    iniparser_set_int(ini, "PSEye:WhiteBalanceR", wbR);
    iniparser_set_int(ini, "PSEye:WhiteBalanceB", wbB);
    iniparser_set_int(ini, "PSEye:WhiteBalanceG", wbG);
    //iniparser_set_int(ini, "PSEye:HFlip", hflip);
    //iniparser_set_int(ini, "PSEye:VFlip", vflip);
    iniparser_save_ini(ini, file);
    dictionary_del(ini);
#endif
}

void camera_control_restore_system_settings(CameraControl* cc, const char* file) {
    // When done using the camera, restore it to its before-psmoveapi settings.
#if !defined(CAMERA_CONTROL_USE_CL_DRIVER) && defined(PSMOVE_USE_PSEYE)
    int NOT_FOUND = -1;
    int AutoAEC = 0;
    int AutoWB = 0;
    int AutoAGC = 0;
    int Gain = 0;
    int Exposure = 0;
    int Sharpness = 0;
    int Contrast = 0;
    int Brightness = 0;
    int wbB = 0;
    int wbG = 0;
    int wbR = 0;

    dictionary* ini = iniparser_load(file);
    AutoAEC = iniparser_getint(ini, "PSEye:AutoAEC", NOT_FOUND);
    AutoWB = iniparser_getint(ini, "PSEye:AutoWB", NOT_FOUND);
    AutoAGC = iniparser_getint(ini, "PSEye:AutoAGC", NOT_FOUND);
    Gain = iniparser_getint(ini, "PSEye:Gain", NOT_FOUND);
    Exposure = iniparser_getint(ini, "PSEye:Exposure", NOT_FOUND);
    Sharpness = iniparser_getint(ini, "PSEye:Sharpness", NOT_FOUND);
    Contrast = iniparser_getint(ini, "PSEye:Contrast", NOT_FOUND);
    Brightness = iniparser_getint(ini, "PSEye:Brightness", NOT_FOUND);
    wbR = iniparser_getint(ini, "PSEye:WhiteBalanceR", NOT_FOUND);
    wbB = iniparser_getint(ini, "PSEye:WhiteBalanceB", NOT_FOUND);
    wbG = iniparser_getint(ini, "PSEye:WhiteBalanceG", NOT_FOUND);
    iniparser_freedict(ini);

    //if (AutoAEC != NOT_FOUND)
    //    ps3eye_set_parameter(cc->eye, PS3EYE_AUTO_EXPOSURE, AutoAEC > 0);
    if (AutoWB != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_AUTO_WHITEBALANCE, AutoWB > 0);
    if (AutoAGC != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_AUTO_GAIN, AutoAGC > 0);
    if (Gain != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_GAIN, Gain);
    if (Exposure != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_EXPOSURE, Exposure);
    if (Contrast != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_CONTRAST, Contrast);
    if (Brightness != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_BRIGHTNESS, Brightness);
    if (wbB != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_BLUEBALANCE, wbB);
    if (wbG != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_GREENBALANCE, wbG);
    if (wbR != NOT_FOUND)
        ps3eye_set_parameter(cc->eye, PS3EYE_REDBALANCE, wbR);

#endif
}

void camera_control_set_parameters(CameraControl* cc, int autoE, int autoG, int autoWB, int exposure, int gain, int wbRed, int wbGreen, int wbBlue, int contrast,
        int brightness) {
#if defined(CAMERA_CONTROL_USE_CL_DRIVER)
    if (autoE >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_AUTO_EXPOSURE, autoE > 0);
    if (autoG >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_AUTO_GAIN, autoG > 0);
    if (autoWB >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_AUTO_WHITEBALANCE, autoWB > 0);
    if (exposure >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_EXPOSURE, round((511 * exposure) / 0xFFFF));
    if (gain >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_GAIN, round((79 * gain) / 0xFFFF));
    if (wbRed >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_WHITEBALANCE_RED, round((255 * wbRed) / 0xFFFF));
    if (wbGreen >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_WHITEBALANCE_GREEN, round((255 * wbGreen) / 0xFFFF));
    if (wbBlue >= 0)
        CLEyeSetCameraParameter(cc->camera, CLEYE_WHITEBALANCE_BLUE, round((255 * wbBlue) / 0xFFFF));
#elif defined(PSMOVE_USE_PSEYE)
    // restart the camera capture with openCv
    if (cc->capture) {
            cvReleaseCapture(&cc->capture);
        }

    //autoE... setAutoExposure not defined in ps3eye.h
    ps3eye_set_parameter(cc->eye, PS3EYE_AUTO_GAIN, autoG > 0);
    ps3eye_set_parameter(cc->eye, PS3EYE_AUTO_WHITEBALANCE, autoWB > 0);
    ps3eye_set_parameter(cc->eye, PS3EYE_EXPOSURE, round(exposure * 255 / 0xFFFF));
    ps3eye_set_parameter(cc->eye, PS3EYE_GAIN, round(gain * 63 / 0xFFFF));
    ps3eye_set_parameter(cc->eye, PS3EYE_REDBALANCE, round(wbRed * 128 / 0xFFFF));
    ps3eye_set_parameter(cc->eye, PS3EYE_BLUEBALANCE, round(wbBlue * 128 / 0xFFFF));
    ps3eye_set_parameter(cc->eye, PS3EYE_GREENBALANCE, round(wbGreen * 128 / 0xFFFF));
    //ps3eye_set_parameter(cc->eye, PS3EYE_CONTRAST, contrast);  // Transform unknown.
    //ps3eye_set_parameter(cc->eye, PS3EYE_BRIGHTNESS, brightness);  // Transform unknown.

    /** The following parameters could be set but are not passed into this function:
     * ps3eye_set_parameter(cc->eye, PS3EYE_SHARPNESS, ??);
     * ps3eye_set_parameter(cc->eye, PS3EYE_HUE, ??);
     * ps3eye_set_parameter(cc->eye, PS3EYE_HFLIP, ??);
     * ps3eye_set_parameter(cc->eye, PS3EYE_VFLIP, ??);
     **/

    int width, height;
    get_metrics(&width, &height);

    cc->capture = cvCaptureFromCAM(cc->cameraID);
    cvSetCaptureProperty(cc->capture, CV_CAP_PROP_FRAME_WIDTH, width);
    cvSetCaptureProperty(cc->capture, CV_CAP_PROP_FRAME_HEIGHT, height);
#endif
}

