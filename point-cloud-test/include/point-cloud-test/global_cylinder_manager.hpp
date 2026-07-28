#ifndef POINT_CLOUD_TEST__GLOBAL_CYLINDER_MANAGER_HPP_
#define POINT_CLOUD_TEST__GLOBAL_CYLINDER_MANAGER_HPP_

#include <cstdint>
#include <cmath>
#include <vector>
#include <algorithm>
#include "point-cloud-test/pcl_processor.h"

namespace point_cloud_test
{

struct TrackedCylinder
{
  int32_t id = 0;
  float center_x = 0.0f;
  float center_y = 0.0f;
  float base_z = 0.0f;
  float radius = 0.0f;
  float height = 0.0f;
  float confidence = 0.0f;
  int32_t seen_count = 0;
  int32_t missed_count = 0;

  float sq_xy_dist(float ox, float oy) const
  {
    float dx = center_x - ox;
    float dy = center_y - oy;
    return dx * dx + dy * dy;
  }
};

class GlobalCylinderManager
{
public:
  GlobalCylinderManager(float max_match_dist = 2.0f)
  : max_match_dist_(max_match_dist)
  {
    tracks_.reserve(128);
  }

  void process(const std::vector<CylinderParams> & detections,
               std::vector<TrackedCylinder> & tracked_out)
  {
    if (detections.empty()) {
      tracked_out = tracks_;
      return;
    }

    const float sq_thresh = max_match_dist_ * max_match_dist_;

    std::vector<bool> track_matched(tracks_.size(), false);

    for (const auto & det : detections) {
      if (!det.isValid) {
        continue;
      }

      int best_idx = -1;
      float best_dist = sq_thresh;

      for (size_t i = 0; i < tracks_.size(); ++i) {
        if (track_matched[i]) {
          continue;
        }

        float d2 = tracks_[i].sq_xy_dist(det.center_x, det.center_y);
        if (d2 < best_dist) {
          best_dist = d2;
          best_idx = static_cast<int>(i);
        }
      }

      if (best_idx >= 0) {
        /* Old Trunk */
        track_matched[best_idx] = true;
        auto & t = tracks_[best_idx];

        t.center_x = t.center_x * 0.7f + det.center_x * 0.3f;
        t.center_y = t.center_y * 0.7f + det.center_y * 0.3f;
        t.base_z = t.base_z * 0.7f + det.base_z * 0.3f;
        t.radius = t.radius * 0.7f + det.radius * 0.3f;
        t.height = t.height * 0.7f + det.height * 0.3f;
        t.confidence = det.confidence;
        t.seen_count++;
        t.missed_count = 0;
      } else {
        /* New Trunk */
        TrackedCylinder new_track;
        new_track.id = next_id_++;
        new_track.center_x = det.center_x;
        new_track.center_y = det.center_y;
        new_track.base_z = det.base_z;
        new_track.radius = det.radius;
        new_track.height = det.height;
        new_track.confidence = det.confidence;
        new_track.seen_count = 1;
        new_track.missed_count = 0;
        tracks_.push_back(new_track);
        track_matched.push_back(true);
      }
    }

    for (size_t i = 0; i < tracks_.size(); ++i) {
      if (!track_matched[i]) {
        tracks_[i].missed_count++;
      }
    }

    tracked_out = tracks_;
  }

private:
  std::vector<TrackedCylinder> tracks_;
  int32_t next_id_ = 0;
  float max_match_dist_ = 2.0f;
};

}  // namespace point_cloud_test

#endif  // POINT_CLOUD_TEST__GLOBAL_CYLINDER_MANAGER_HPP_