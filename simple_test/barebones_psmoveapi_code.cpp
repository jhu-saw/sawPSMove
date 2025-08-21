// psmove_inspect.cpp
// Minimal inspector for PS Move API + (optional) camera tracker.
// Build (Linux):
//   export PSMOVE_ROOT=~/psmoveapi
//   export PSMOVE_BUILD=$PSMOVE_ROOT/build
//   g++ -O2 psmove_inspect.cpp \
//     -I$PSMOVE_ROOT/include -I$PSMOVE_BUILD -L$PSMOVE_BUILD \
//     -o psmove_inspect -lpsmoveapi -lpthread
//
// With camera tracker (still no OpenCV headers in your code):
//   g++ -O2 psmove_inspect.cpp \
//     -I$PSMOVE_ROOT/include -I$PSMOVE_BUILD -L$PSMOVE_BUILD \
//     -o psmove_inspect \
//     -lpsmoveapi -lpsmoveapi_tracker \
//     `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
//     -lpthread
//
// Run:
//   LD_LIBRARY_PATH=$PSMOVE_BUILD ./psmove_inspect
//   LD_LIBRARY_PATH=$PSMOVE_BUILD ./psmove_inspect --camera
//
// Controls:
//   PS button   -> quit
//   MOVE button -> reset orientation (zero yaw)

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <unistd.h>

// --- psmoveapi C headers
extern "C" {
#include "psmove.h"
#include "psmove_tracker.h"
}

// --- psmoveapi compatibility shims ---
// Older builds used Btn_TRIGGER; newer ones use Btn_T.
#ifndef Btn_T
# ifdef Btn_TRIGGER
#  define Btn_T Btn_TRIGGER
# endif
#endif
// Some builds don’t define PSMove_True/False; API accepts int/bool.
#ifndef PSMove_True
# define PSMove_True 1
#endif
#ifndef PSMove_False
# define PSMove_False 0
#endif
// --------------------------------------

static const char *btn_name(unsigned int b) {
    switch (b) {
        case Btn_TRIANGLE:  return "TRIANGLE";
        case Btn_CIRCLE:    return "CIRCLE";
        case Btn_CROSS:     return "CROSS";
        case Btn_SQUARE:    return "SQUARE";
        case Btn_SELECT:    return "SELECT";
        case Btn_START:     return "START";
        case Btn_PS:        return "PS";
        case Btn_MOVE:      return "MOVE";
        case Btn_T:         return "TRIGGER";
        default:            return "?";
    }
}

int main(int argc, char** argv) {
    bool use_camera = (argc > 1 && std::strcmp(argv[1], "--camera") == 0);

    std::puts("[init] psmove_connect()");
    PSMove *move = psmove_connect();    // connect first available controller
    if (!move) {
        std::fprintf(stderr, "ERROR: No PS Move controller found\n");
        return 1;
    }

    int connected_count = psmove_count_connected();
    int battery = psmove_get_battery(move);
    int connection_type = psmove_connection_type(move);
    std::printf("[info] connected controllers: %d, battery: %d, connection: %d\n",
                connected_count, battery, connection_type);

    // Enable orientation (IMU fusion)
    std::puts("[call] psmove_enable_orientation(true)");
    psmove_enable_orientation(move, true); // equivalent to PSMove_True

    // Optional tracker (no OpenCV includes here)
    PSMoveTracker *tracker = nullptr;
    if (use_camera) {
        std::puts("[init] psmove_tracker_new()");
        tracker = psmove_tracker_new();
        if (!tracker) {
            std::fprintf(stderr, "ERROR: tracker init failed\n");
            psmove_disconnect(move);
            return 2;
        }
        std::puts("[loop] psmove_tracker_enable(...), waiting for Tracker_CALIBRATED");
        while (psmove_tracker_enable(tracker, move) != Tracker_CALIBRATED) {
            // keep retrying until calibrated
        }
        std::puts("[ok] tracker calibrated");
    }

    std::puts("[loop] polling controller with psmove_poll()");
    std::puts("       press PS to quit, MOVE to reset orientation");

    float q0,q1,q2,q3;
    bool quit = false;

    while (!quit) {
        // Drain all pending HID events
        while (psmove_poll(move)) {
            unsigned int buttons = psmove_get_buttons(move);
            if (buttons) {
                // Log which buttons are down (bitmask scan)
                for (unsigned int b = 1; b <= Btn_T; b <<= 1) {
                    if (buttons & b) {
                        std::printf("[event] psmove_get_buttons() -> %s\n", btn_name(b));
                    }
                }
            }

            if (buttons & Btn_PS) {
                std::puts("[quit] PS pressed");
                quit = true;
                break;
            }
            if (buttons & Btn_MOVE) {
                std::puts("[call] psmove_reset_orientation()");
                psmove_reset_orientation(move);
            }

            // Orientation quaternion (w,x,y,z)
            psmove_get_orientation(move, &q0, &q1, &q2, &q3);
            std::printf("[data] psmove_get_orientation() -> q=[% .4f % .4f % .4f % .4f]\n",
                        q0,q1,q2,q3);

            // Optional: analog trigger value (0..255)
            unsigned char t = psmove_get_trigger(move);
            std::printf("[data] psmove_get_trigger() -> %u\n", (unsigned)t);

            // Optional: 2D tracker position (if camera enabled)
            if (use_camera) {
                float x=0.f, y=0.f, r=0.f;
                psmove_tracker_get_position(tracker, move, &x, &y, &r);
                std::printf("[data] psmove_tracker_get_position() -> x=%.1f y=%.1f r=%.1f\n",
                            x,y,r);
            }
        }

        if (use_camera) {
            // Grab latest frame + update tracking (no OpenCV window shown)
            psmove_tracker_update_image(tracker);
            psmove_tracker_update(tracker, nullptr);
        }

        // Light sleep to avoid spinning
        usleep(1000 * 5); // 5 ms
    }

    if (tracker) {
        std::puts("[cleanup] psmove_tracker_free()");
        psmove_tracker_free(tracker);
    }
    std::puts("[cleanup] psmove_disconnect()");
    psmove_disconnect(move);
    return 0;
}
