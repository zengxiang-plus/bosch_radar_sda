#include "report_msgs_manager.h"

namespace driver {
namespace radar {
PLUSAI_DECLARE_bool(enable_pub_parsed_info);

bool ReportMsgsManager::Init() {
    radar_parsed_array_.tracks.reserve(512);
    radar_track_array_.tracks.reserve(512);

    std::string radar_parsed_topic = frame_id_ + "/radar_parsed";
    std::string radar_track_topic = frame_id_ + "/radar_tracks";

    if (FLAGS_enable_pub_parsed_info) {
        radar_parsed_pub_ = drive::common::ipc::advertise <
                                bosch_radar_msgs::CR5TPRadarParsedArray > (node_handle_, radar_parsed_topic, 1);
        if(!radar_parsed_pub_) {
            LOG(ERROR) << "Failed to advertise radar parsed information...";
            return -1;
        }
    }

    radar_track_pub_ = drive::common::ipc::advertise <
                                radar_msgs::RadarTrackArray > (node_handle_, radar_track_topic, 1);
    if (!radar_track_pub_) {
        LOG(ERROR) << "Failed to advertise radar track information...";
        return -1;
    }
    return 0;
}

void ReportMsgsManager::setObjectSize(int object_size, const ros::Time& msg_stamp) {
    object_size_ = object_size;
    object_added_counter_ = 0;
    msg_stamp_ = msg_stamp;
}

void ReportMsgsManager::addOneObject() {
    ++object_added_counter_;
}

bool ReportMsgsManager::isFrameComplete() {
    return object_size_ >= 0 && (object_added_counter_ >= object_size_);
}

void ReportMsgsManager::resetFrame() {
    object_size_ = -1;
    object_added_counter_ = -1;

    /* here the tracks are not destroyed. 
       So Pre-matched memory will not be released 
       And we don't need to reallocate memory  */
    radar_parsed_array_.tracks.clear();
    radar_track_array_.tracks.clear();

}

void ReportMsgsManager::publishData() {
    radar_track_array_.header.stamp = msg_stamp_;
    radar_track_array_.header.frame_id = frame_id_;
    
    radar_track_pub_.publish(radar_track_array_);

    if(FLAGS_enable_pub_parsed_info) {
        radar_parsed_array_.header = radar_track_array_.header;
        radar_parsed_pub_.publish(radar_parsed_array_);
    }

    resetFrame();
}

}  // namespace radar
}  // namespace driver
