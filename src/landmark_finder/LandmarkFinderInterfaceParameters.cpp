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
    getParam(nh, "threshold", cfg.threshold);
    getParam(nh, "tight_filter_size", cfg.tight_filter_size);
    getParam(nh, "wide_filter_size", cfg.wide_filter_size);
    getParam(nh, "maxRadiusForPixelCluster", cfg.maxRadiusForPixelCluster);
    getParam(nh, "minPixelForCluster", cfg.minPixelForCluster);
    getParam(nh, "maxPixelForCluster", cfg.maxPixelForCluster);
    getParam(nh, "maxRadiusForCluster", cfg.maxRadiusForCluster);
    getParam(nh, "minPointsPerLandmark", cfg.minPointsPerLandmark);
    getParam(nh, "maxPointsPerLandmark", cfg.maxPointsPerLandmark);
    getParam(nh, "maxCornerHypotheses", cfg.maxCornerHypotheses);
    getParam(nh, "cornerHypothesesCutoff", cfg.cornerHypothesesCutoff);

}

void LandmarkFinderInterfaceParameters::fromConfig(const LandmarkFinderConfig& config, const uint32_t&) {
    cfg = config;
}
}
