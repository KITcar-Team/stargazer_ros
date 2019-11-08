#include "LandmarkFinderInterfaceParameters.h"

#include "../ros_utils.h"

namespace stargazer_ros_tool {

void LandmarkFinderInterfaceParameters::fromNodeHandle(const ros::NodeHandle& nh) {

    getParam(nh, "map_config", map_config);
    getParam(nh, "debug_mode", cfg.debug_mode);

    // Topics
    getParam(nh, "landmark_topic", landmark_topic);
    getParam(nh, "undistorted_image_topic", undistorted_image_topic);

    // Detection Parameters
    getParam(nh, "maxRadiusForCluster", cfg.maxRadiusForCluster);
    getParam(nh, "minPointsPerLandmark", cfg.minPointsPerLandmark);
    getParam(nh, "maxPointsPerLandmark", cfg.maxPointsPerLandmark);

    getParam(nh, "blobFilterByArea", cfg.blobFilterByArea);
    getParam(nh, "blobFilterByCircularity", cfg.blobFilterByCircularity);
    getParam(nh, "blobFilterByConvexity", cfg.blobFilterByConvexity);
    getParam(nh, "blobFilterByInertia", cfg.blobFilterByInertia);
    getParam(nh, "blobMaxArea", cfg.blobMaxArea);
    getParam(nh, "blobMaxCircularity", cfg.blobMaxCircularity);
    getParam(nh, "blobMaxConvexity", cfg.blobMaxConvexity);
    getParam(nh, "blobMaxInertiaRatio", cfg.blobMaxInertiaRatio);
    getParam(nh, "blobMaxThreshold", cfg.blobMaxThreshold);
    getParam(nh, "blobMinArea", cfg.blobMinArea);
    getParam(nh, "blobMinCircularity", cfg.blobMinCircularity);
    getParam(nh, "blobMinConvexity", cfg.blobMinConvexity);
    getParam(nh, "blobMinDistBetweenBlobs", cfg.blobMinDistBetweenBlobs);
    getParam(nh, "blobMinInertiaRatio", cfg.blobMinInertiaRatio);
    getParam(nh, "blobMinRepeatability", cfg.blobMinRepeatability);
    getParam(nh, "blobMinThreshold", cfg.blobMinThreshold);
    getParam(nh, "blobThresholdStep", cfg.blobThresholdStep);
}

void LandmarkFinderInterfaceParameters::fromConfig(const LandmarkFinderConfig& config, const uint32_t&) {
    cfg = config;
}
}
