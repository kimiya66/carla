// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/WaypointGenerator.h"

#include "carla/road/Map.h"

namespace carla {
namespace road {

  using namespace carla::road::element;

  // ===========================================================================
  // -- Static local methods ---------------------------------------------------
  // ===========================================================================

  template <typename T>
  static std::vector<T> ConcatVectors(std::vector<T> dst, std::vector<T> src) {
    if (src.size() > dst.size()) {
      return ConcatVectors(src, dst);
    }
    dst.insert(
        dst.end(),
        std::make_move_iterator(src.begin()),
        std::make_move_iterator(src.end()));
    return dst;
  }

  template <typename FuncT>
  static void ForEachDrivableLane(const RoadSegment &road, double s, FuncT &&func) {
    const auto info = road.GetInfo<RoadInfoLane>(s);
    DEBUG_ASSERT(info != nullptr);
    for (auto &&lane_id : info->getLanesIDs(RoadInfoLane::which_lane_e::Both)) {
      if (info->getLane(lane_id)->_type == "driving") {
        func(lane_id);
      }
    }
  }

  // ===========================================================================
  // -- WaypointGenerator ------------------------------------------------------
  // ===========================================================================

  std::vector<Waypoint> WaypointGenerator::GetSuccessors(const Waypoint &waypoint) {
    auto &map = waypoint._map;
    const auto this_lane_id = waypoint.GetLaneId();
    const auto this_road_id = waypoint.GetRoadId();

    const auto &next_lanes =
        this_lane_id <= 0 ?
            waypoint.GetRoadSegment().GetNextLane(this_lane_id) :
            waypoint.GetRoadSegment().GetPrevLane(this_lane_id);

    if (next_lanes.empty()) {
      log_error("road id =", this_road_id, "lane id =", this_lane_id, ": missing next lanes");
    }

    std::vector<Waypoint> result;
    result.reserve(next_lanes.size());
    for (auto &&pair : next_lanes) {
      const auto lane_id = pair.first;
      const auto road_id = pair.second;
      const auto road = map->GetData().GetRoad(road_id);
      DEBUG_ASSERT(lane_id != 0);
      DEBUG_ASSERT(road != nullptr);
      const auto distance = lane_id < 0 ? 0.0 : road->GetLength();
      result.push_back(Waypoint(map, road_id, lane_id, distance));
    }
    return result;
  }

  std::vector<Waypoint> WaypointGenerator::GetNext(
      const Waypoint &waypoint,
      double distance) {
    auto &map = waypoint._map;
    const auto this_road_id = waypoint.GetRoadId();
    const auto this_lane_id = waypoint.GetLaneId();

    DEBUG_ASSERT(this_lane_id != 0);

    double distance_on_next_segment;

    if (this_lane_id <= 0) {
      // road goes forward.
      const auto total_distance = waypoint._dist + distance;
      const auto road_length = waypoint.GetRoadSegment().GetLength();
      if (total_distance <= road_length) {
        return { Waypoint(map, this_road_id, this_lane_id, total_distance) };
      }
      distance_on_next_segment = total_distance - road_length;
    } else {
      // road goes backward.
      const auto total_distance = waypoint._dist - distance;
      if (total_distance >= 0.0) {
        return { Waypoint(map, this_road_id, this_lane_id, total_distance) };
      }
      distance_on_next_segment = std::abs(total_distance);
    }

    std::vector<Waypoint> result;
    for (auto &&next_waypoint : GetSuccessors(waypoint)) {
      result = ConcatVectors(result, GetNext(next_waypoint, distance_on_next_segment));
    }
    return result;
  }

  boost::optional<Waypoint> WaypointGenerator::GetRight(const Waypoint &waypoint) {
    auto &map = waypoint._map;
    const auto this_road_id = waypoint.GetRoadId();
    const auto this_lane_id = waypoint.GetLaneId();

    DEBUG_ASSERT(this_lane_id != 0);

    const int new_lane_id = (this_lane_id <= 0) ? this_lane_id - 1 : this_lane_id + 1;

    // check if that lane id exists on this distance
    const auto road = map->GetData().GetRoad(this_road_id);
    const auto mark_record_vector = road->GetRoadInfoMarkRecord(waypoint._dist);
    for (auto &&mark_record : mark_record_vector) {
      // find if the lane id exists
      if (mark_record->GetLaneId() == new_lane_id) {
        return Waypoint(map, this_road_id, new_lane_id, waypoint._dist);
      }
    }
    return boost::optional<Waypoint>();
  }

  boost::optional<Waypoint> WaypointGenerator::GetLeft(const Waypoint &waypoint) {
    auto &map = waypoint._map;
    const auto this_road_id = waypoint.GetRoadId();
    const auto this_lane_id = waypoint.GetLaneId();

    DEBUG_ASSERT(this_lane_id != 0);

    int new_lane_id;
    if (this_lane_id > 0) {
      // road goes backward: decrease the lane id while avoiding returning lane 0
      new_lane_id = this_lane_id - 1 == 0 ? -1 : this_lane_id - 1;
    } else {
      // road goes forward: increasing the lane id while avoiding returning lane 0
      new_lane_id = this_lane_id + 1 == 0 ? 1 : this_lane_id + 1;
    }

    // check if that lane id exists on this distance
    const auto road = map->GetData().GetRoad(this_road_id);
    const auto mark_record_vector = road->GetRoadInfoMarkRecord(waypoint._dist);
    for (auto &&mark_record : mark_record_vector) {
      // find if the lane id exists
      if (mark_record->GetLaneId() == new_lane_id) {
        return Waypoint(map, this_road_id, new_lane_id, waypoint._dist);
      }
    }
    return boost::optional<Waypoint>();
  }

  std::vector<Waypoint> WaypointGenerator::GenerateAll(
      const Map &map,
      const double distance) {
    std::vector<Waypoint> result;
    for (auto &&road_segment : map.GetData().GetRoadSegments()) {
      /// @todo Should distribute them equally along the segment?
      for (double s = 0.0; s < road_segment.GetLength(); s += distance) {
        ForEachDrivableLane(road_segment, s, [&](auto lane_id) {
          result.push_back(Waypoint(map.shared_from_this(), road_segment.GetId(), lane_id, s));
        });
      }
    }
    return result;
  }

  std::vector<Waypoint> WaypointGenerator::GenerateLaneBegin(
      const Map &map) {
    std::vector<Waypoint> result;
    for (auto &&road_segment : map.GetData().GetRoadSegments()) {
      ForEachDrivableLane(road_segment, 0.0, [&](auto lane_id) {
        auto distance = lane_id < 0 ? 0.0 : road_segment.GetLength();
        auto this_waypoint = Waypoint(
            map.shared_from_this(),
            road_segment.GetId(),
            lane_id,
            distance);
        result.push_back(this_waypoint);
      });
    }
    return result;
  }

  std::vector<Waypoint> WaypointGenerator::GenerateLaneEnd(
      const Map &map) {
    std::vector<Waypoint> result;
    for (auto &&road_segment : map.GetData().GetRoadSegments()) {
      ForEachDrivableLane(road_segment, 0.0, [&](auto lane_id) {
        auto distance = lane_id > 0 ? 0.0 : road_segment.GetLength();
        auto this_waypoint = Waypoint(
            map.shared_from_this(),
            road_segment.GetId(),
            lane_id,
            distance);
        result.push_back(this_waypoint);
      });
    }
    return result;
  }

  std::vector<std::pair<Waypoint, Waypoint>> WaypointGenerator::GenerateTopology(
      const Map &map) {
    std::vector<std::pair<Waypoint, Waypoint>> result;
    for (auto &&road_segment : map.GetData().GetRoadSegments()) {
      ForEachDrivableLane(road_segment, 0.0, [&](auto lane_id) {
        auto distance = lane_id < 0 ? 0.0 : road_segment.GetLength();
        auto this_waypoint = Waypoint(
            map.shared_from_this(),
            road_segment.GetId(),
            lane_id,
            distance);
        for (auto &&successor : GetSuccessors(this_waypoint)) {
          result.push_back({this_waypoint, successor});
        }
      });
    }
    return result;
  }

} // namespace road
} // namespace carla
