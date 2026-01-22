# pragma once
# include <array>
# include <optional>
# include <cstddef>

namespace constants {
    // Define a datatype for tag pose
    struct TagPose {
        int tagID;
        float x, y, z;
        float yaw, pitch, roll;
    };

    // Define a datatype to hold camera info
    struct Camera {
        float fx, fy, cx, cy; // Intrinsics
        float x, y, z, roll, pitch, yaw; // Pose relative to robot
    };

    // Define the tag size (in meters)
    constexpr float tag_size = 0.1651; // [PLACEHOLDER]

    // Create an array of all the cameras attached to the Pi
    constexpr std::array<Camera, 2> Cameras = {{
        /* camera 0 */ {0.5f, 0.5f, 0.5f, 0.f, 0.f, 0.f, /* <--- intrinsics | pose ---> */ 1000.f, 1000.f, 960.f, 540.f}, // [PLACEHOLDER]
        /* camera 1 */ {0.5f, 0.5f, 0.5f, 0.f, 0.f, 0.f, /* <--- intrinsics | pose ---> */ 1000.f, 1000.f, 960.f, 540.f} // [PLACEHOLDER
    }};

    // MUST be sorted by id in strictly increasing order [IMPORTANT] <---------!!!!!!!!
    constexpr std::array<TagPose, 2> tagPosesGlobal = {{
        {0, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f},
        {1, 1.f, 1.f, 1.f, 0.f, 0.f, 0.f}
    }}; // [PLACEHOLDER]

    // Binary-search lookup by tag ID
    constexpr std::optional<TagPose> getTagPose(int id){
        std::size_t left = 0;
        std::size_t right = tagPosesGlobal.size();

        while (left < right) {
            std::size_t mid = (left + right) / 2;
            int midID = tagPosesGlobal[mid].tagID;

            if (midID == id) {
                return tagPosesGlobal[mid];
            }
            if (midID < id) {
                left = mid + 1;
            } else {
                right = mid;
            }
        }
        
        return std::nullopt;
    }
}