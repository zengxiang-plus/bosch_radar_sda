#pragma once

#include <unordered_map>
#include <mutex>

#include <glog/logging.h>
#include <ros/ros.h>
#include <radar_msgs/RadarTrackArray.h>
#include <radar_msgs/RadarTrack.h>
#include <ipc/publisher.h>

#include "bosch_radar_msgs/CR5TPRadarParsedArray.h"
#include "bosch_radar_msgs/CR5TPRadarParsed.h"

namespace driver {
namespace radar {

typedef bosch_radar_msgs::CR5TPRadarParsed RadarParsedObject;
typedef radar_msgs::RadarTrack RadarTrackObject;
typedef bosch_radar_msgs::CR5TPRadarParsedArray RadarParsedArray;
typedef radar_msgs::RadarTrackArray RadarTrackArray;

class ReportMsgsManager {

  public:
    ReportMsgsManager(std::string frame_id) : frame_id_(frame_id) {
    }
    ReportMsgsManager(const ReportMsgsManager&) = delete;
    ReportMsgsManager& operator=(const ReportMsgsManager&) = delete;
    ~ReportMsgsManager() = default;

    bool Init();

    /* some check function for publishing topic */
    void setObjectSize(int object_size, const ros::Time& msg_stamp);
    void addOneObject(); 
    bool isFrameComplete();
    void resetFrame();
    void publishData();

    /* use reference to avoid copy cost and therefore no use const */
    RadarTrackArray& GetRadarTrackArray() {
        return radar_track_array_;
    }
    RadarParsedArray& GetRadarParsedArray() {
        return radar_parsed_array_;
    }

  private:
    std::string const frame_id_;
    ros::NodeHandle node_handle_;
    ros::Time msg_stamp_;

    bosch_radar_msgs::CR5TPRadarParsedArray radar_parsed_array_;
    drive::common::ipc::Publisher radar_parsed_pub_;

    radar_msgs::RadarTrackArray radar_track_array_;
    drive::common::ipc::Publisher radar_track_pub_;

    int object_size_ = -1;
    int object_added_counter_ = -1;
};

}  // namespace radar
}  // namespace driver
