# include "CameraDevice.h"
# include "constants.h"

# include <iostream>

int main() {
    camera::CameraDevice cam0[0, 1280, 1080]
    camera::CameraDevice cam0[1, 1280, 1080]

    cam0.start();
    cam1.start();

    while (true) {
        auto frame0 = cam0.getLatestFrame();
        auto frame1 = cam1.getLatestFrame();

        if (!frame0.empty() && !frame1.empty()) {
            // Proccessing
        }
    }

    return 0;
}