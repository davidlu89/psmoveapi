
 /**
 * PS Move API - An interface for the PS Move Motion Controller
 * Copyright (c) 2012 Thomas Perl <m@thp.io>
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


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#include "psmove.h"
#if defined(PSMOVE_WITH_MADGWICK_AHRS)
#include <math.h>
#endif

int
main(int argc, char* argv[])
{
    PSMove *move;

    move = psmove_connect();

    if (move == NULL) {
        fprintf(stderr, "Could not connect to controller.\n");
        return EXIT_FAILURE;
    }
    
    assert(psmove_has_calibration(move));
    
#if defined(PSMOVE_WITH_MADGWICK_AHRS)
    psmove_enable_orientation(move, PSMove_True);
    assert(psmove_has_orientation(move));
    float xAngle, yAngle, zAngle;
#endif

    if (psmove_connection_type(move) == Conn_Bluetooth) {
        float ax, ay, az, gx, gy, gz, mx, my, mz;

        while (1) {
            int res = psmove_poll(move);
            if (res) {
                psmove_get_accelerometer_frame(move, Frame_SecondHalf,
                        &ax, &ay, &az);
                psmove_get_gyroscope_frame(move, Frame_SecondHalf,
                        &gx, &gy, &gz);
                
                psmove_get_magnetometer_vector(move,
                                               &mx, &my, &mz);

                printf("A: %6.3f %6.3f %6.3f   ", ax, ay, az);
                printf("G: %7.3f %7.3f %7.3f   ", gx, gy, gz);
                printf("M: %6.3f %6.3f %6.3f", mx, my, mz);
                
#if defined(PSMOVE_WITH_MADGWICK_AHRS)
                psmove_get_angles(move, &xAngle, &yAngle, &zAngle);
                printf("   E: %4.2f %4.2f %4.2f", xAngle, yAngle, zAngle);
#endif
                printf("\n");
            }
        }
    }

    psmove_disconnect(move);

    return EXIT_SUCCESS;
}

