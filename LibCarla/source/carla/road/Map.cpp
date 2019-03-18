// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "carla/road/Map.h"

#include <stdexcept>
#include "carla/Exception.h"

#include "carla/road/element/LaneCrossingCalculator.h"

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

  /// Return a waypoint for each drivable lane on each lane section of @a road.
  template <typename FuncT>
  static void ForEachDrivableLane(const Road &road, FuncT &&func) {
    for (const auto &lane_section : road.GetLaneSections()) {
      for (const auto &pair : lane_section.GetLanes()) {
        const auto &lane = pair.second;
        if (lane.GetType() == "driving") {
          func(Waypoint{road.GetId(), lane.GetId(), lane_section.GetDistance()});
        }
      }
    }
  }

  // ===========================================================================
  // -- Map --------------------------------------------------------------------
  // ===========================================================================

  Waypoint Map::GetClosestWaypointOnRoad(const geom::Location &pos) const {
    // max_nearests represents the max nearests roads
    // where we will search for nearests lanes
    constexpr int max_nearests = 10;
    // in case that map has less than max_nearests lanes,
    // we will use the maximum lanes
    const int max_nearest_allowed = _data.GetRoadCount() < max_nearests ?
        _data.GetRoadCount() : max_nearests;

    double nearest_dist[max_nearests];
    std::fill(nearest_dist, nearest_dist + max_nearest_allowed,
        std::numeric_limits<double>::max());

    RoadId ids[max_nearests];
    std::fill(ids, ids + max_nearest_allowed, 0);

    double dists[max_nearests];
    std::fill(dists, dists + max_nearest_allowed, 0.0);

    for (const auto &road_pair : _data.GetRoads()) {
      const auto road = &road_pair.second;
      const auto current_dist = road->GetNearestPoint(pos);

      for (int i = 0; i < max_nearest_allowed; ++i) {
        if (current_dist.second < nearest_dist[i]) {
          // reorder nearest_dist
          for (int j = max_nearest_allowed - 1; j > i; --j) {
            nearest_dist[j] = nearest_dist[j - 1];
            ids[j] = ids[j - 1];
            dists[j] = dists[j - 1];
          }
          nearest_dist[i] = current_dist.second;
          ids[i] = road->GetId();
          dists[i] = current_dist.first;

          break;
        }
      }
    }

    // search for the nearest lane in nearest_dist
    // Waypoint waypoint;
    // float nearest_lane_dist = std::numeric_limits<float>::max();
    // for (int i = 0; i < max_nearest_allowed; ++i) {
    //   auto lane_dist = _data.GetRoad(ids[i])->GetNearestLane(dists[i], pos);

    //   if (lane_dist.second < nearest_lane_dist) {
    //     nearest_lane_dist = lane_dist.second;
    //     waypoint.lane_id = lane_dist.first;
    //     waypoint.road_id = ids[i];
    //     waypoint.dist = dists[i];
    //   }
    // }

    // DEBUG_ASSERT(_dist <= _map->GetData().GetRoad(_road_id)->GetLength());
    // DEBUG_ASSERT(_lane_id != 0);

    return {}; //Waypoint(shared_from_this(), loc);
  }

  boost::optional<Waypoint> Map::GetWaypoint(const geom::Location &/* loc */) const {
    // Waypoint w = Waypoint(shared_from_this(), loc);
    // auto d = geom::Math::Distance2D(w.ComputeTransform().location, loc);
    // const auto inf = _data.GetRoad(w._road_id)->GetInfo<RoadInfoLane>(w._dist);

    // if (d < inf->getLane(w._lane_id)->_width * 0.5) {
    //   return w;
    // }
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  std::vector<element::LaneMarking> Map::CalculateCrossedLanes(
      const geom::Location &/* origin */,
      const geom::Location &/* destination */) const {
    // return element::LaneCrossingCalculator::Calculate(*this, origin, destination);
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  std::vector<Waypoint> Map::GetSuccessors(const Waypoint &/* waypoint */) const {
    // auto &map = waypoint._map;
    // const auto this_lane_id = waypoint.GetLaneId();
    // const auto this_road_id = waypoint.GetRoadId();

    // const auto &next_lanes =
    //     this_lane_id <= 0 ?
    //         waypoint.GetRoadSegment().GetNextLane(this_lane_id) :
    //         waypoint.GetRoadSegment().GetPrevLane(this_lane_id);

    // if (next_lanes.empty()) {
    //   log_error("road id =", this_road_id, "lane id =", this_lane_id, ": missing next lanes");
    // }

    // std::vector<Waypoint> result;
    // result.reserve(next_lanes.size());
    // for (auto &&pair : next_lanes) {
    //   const auto lane_id = pair.first;
    //   const auto road_id = pair.second;
    //   const auto road = map->GetData().GetRoad(road_id);
    //   DEBUG_ASSERT(lane_id != 0);
    //   DEBUG_ASSERT(road != nullptr);
    //   const auto distance = lane_id < 0 ? 0.0 : road->GetLength();
    //   result.push_back(Waypoint(map, road_id, lane_id, distance));
    // }
    // return result;
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  std::vector<Waypoint> Map::GetNext(
      const Waypoint &/* waypoint */,
      float /* distance */) const {
    // auto &map = waypoint._map;
    // const auto this_road_id = waypoint.GetRoadId();
    // const auto this_lane_id = waypoint.GetLaneId();

    // DEBUG_ASSERT(this_lane_id != 0);

    // float distance_on_next_segment;

    // if (this_lane_id <= 0) {
    //   // road goes forward.
    //   const auto total_distance = waypoint._dist + distance;
    //   const auto road_length = waypoint.GetRoadSegment().GetLength();
    //   if (total_distance <= road_length) {
    //     return { Waypoint(map, this_road_id, this_lane_id, total_distance) };
    //   }
    //   distance_on_next_segment = total_distance - road_length;
    // } else {
    //   // road goes backward.
    //   const auto total_distance = waypoint._dist - distance;
    //   if (total_distance >= 0.0) {
    //     return { Waypoint(map, this_road_id, this_lane_id, total_distance) };
    //   }
    //   distance_on_next_segment = std::abs(total_distance);
    // }

    // std::vector<Waypoint> result;
    // for (auto &&next_waypoint : GetSuccessors(waypoint)) {
    //   result = ConcatVectors(result, GetNext(next_waypoint, distance_on_next_segment));
    // }
    // return result;
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  boost::optional<Waypoint> Map::GetRight(const Waypoint &/* waypoint */) const {
    // auto &map = waypoint._map;
    // const auto this_road_id = waypoint.GetRoadId();
    // const auto this_lane_id = waypoint.GetLaneId();

    // DEBUG_ASSERT(this_lane_id != 0);

    // const int new_lane_id = (this_lane_id <= 0) ? this_lane_id - 1 : this_lane_id + 1;

    // // check if that lane id exists on this distance
    // const auto road = map->GetData().GetRoad(this_road_id);
    // const auto mark_record_vector = road->GetRoadInfoMarkRecord(waypoint._dist);
    // for (auto &&mark_record : mark_record_vector) {
    //   // find if the lane id exists
    //   if (mark_record->GetLaneId() == new_lane_id) {
    //     return Waypoint(map, this_road_id, new_lane_id, waypoint._dist);
    //   }
    // }
    // return boost::optional<Waypoint>();
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  boost::optional<Waypoint> Map::GetLeft(const Waypoint &/* waypoint */) const {
    // auto &map = waypoint._map;
    // const auto this_road_id = waypoint.GetRoadId();
    // const auto this_lane_id = waypoint.GetLaneId();

    // DEBUG_ASSERT(this_lane_id != 0);

    // int new_lane_id;
    // if (this_lane_id > 0) {
    //   // road goes backward: decrease the lane id while avoiding returning lane 0
    //   new_lane_id = this_lane_id - 1 == 0 ? -1 : this_lane_id - 1;
    // } else {
    //   // road goes forward: increasing the lane id while avoiding returning lane 0
    //   new_lane_id = this_lane_id + 1 == 0 ? 1 : this_lane_id + 1;
    // }

    // // check if that lane id exists on this distance
    // const auto road = map->GetData().GetRoad(this_road_id);
    // const auto mark_record_vector = road->GetRoadInfoMarkRecord(waypoint._dist);
    // for (auto &&mark_record : mark_record_vector) {
    //   // find if the lane id exists
    //   if (mark_record->GetLaneId() == new_lane_id) {
    //     return Waypoint(map, this_road_id, new_lane_id, waypoint._dist);
    //   }
    // }
    // return boost::optional<Waypoint>();
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  std::vector<Waypoint> Map::GenerateWaypoints(
      const float /* distance */) const {
    // std::vector<Waypoint> result;
    // for (auto &&road_segment : map.GetData().GetRoadSegments()) {
    //   /// @todo Should distribute them equally along the segment?
    //   for (float s = 0.0; s < road_segment.GetLength(); s += distance) {
    //     ForEachDrivableLane(road_segment, s, [&](auto lane_id) {
    //       result.push_back(Waypoint(map.shared_from_this(), road_segment.GetId(), lane_id, s));
    //     });
    //   }
    // }
    // return result;
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  std::vector<Waypoint> Map::GenerateLaneBegin() const {
    // std::vector<Waypoint> result;
    // for (auto &&road_segment : map.GetData().GetRoadSegments()) {
    //   ForEachDrivableLane(road_segment, 0.0, [&](auto lane_id) {
    //     auto distance = lane_id < 0 ? 0.0 : road_segment.GetLength();
    //     auto this_waypoint = Waypoint(
    //         map.shared_from_this(),
    //         road_segment.GetId(),
    //         lane_id,
    //         distance);
    //     result.push_back(this_waypoint);
    //   });
    // }
    // return result;
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  std::vector<Waypoint> Map::GenerateLaneEnd() const {
    // std::vector<Waypoint> result;
    // for (auto &&road_segment : map.GetData().GetRoadSegments()) {
    //   ForEachDrivableLane(road_segment, 0.0, [&](auto lane_id) {
    //     auto distance = lane_id > 0 ? 0.0 : road_segment.GetLength();
    //     auto this_waypoint = Waypoint(
    //         map.shared_from_this(),
    //         road_segment.GetId(),
    //         lane_id,
    //         distance);
    //     result.push_back(this_waypoint);
    //   });
    // }
    // return result;
    throw_exception(std::runtime_error("not implemented"));
    return {};
  }

  std::vector<std::pair<Waypoint, Waypoint>> Map::GenerateTopology() const {
    std::vector<std::pair<Waypoint, Waypoint>> result;
    for (const auto &pair : _data.GetRoads()) {
      const auto &road = pair.second;
      ForEachDrivableLane(road, [&](auto &&waypoint) {
        for (auto &&successor : GetSuccessors(waypoint)) {
          result.push_back({waypoint, successor});
        }
      });
    }
    return result;
  }

} // namespace road
} // namespace carla
