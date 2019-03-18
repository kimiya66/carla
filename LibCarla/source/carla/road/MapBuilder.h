// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "carla/road/Map.h"

#include <boost/optional.hpp>

#include <map>

namespace carla {
namespace road {

  class MapBuilder {
  public:

    boost::optional<Map> Build();

    // called from road parser
    void AddRoad(
        const uint32_t road_id,
        const std::string name,
        const double length,
        const int32_t junction_id,
        const int32_t predecessor,
        const int32_t successor);

    void AddRoadSection(
        const uint32_t road_id,
        geom::CubicPolynomial section);

    void AddRoadSectionLane(
        const uint32_t road_id,
        const uint32_t section_index,
        const int32_t lane_id,
        const std::string lane_type,
        const bool lane_level,
        const int32_t predecessor,
        const int32_t successor);

    void SetRoadTypeSpeed(
        const uint32_t road_id,
        const double s,
        const std::string type,
        const double max,
        const std::string unit);

    // called from geometry parser
    void AddRoadGeometryLine(
        carla::road::Road *road,
        const double s,
        const double x,
        const double y,
        const double hdg,
        const double length);

    void AddRoadGeometryArc(
        carla::road::Road *road,
        const double s,
        const double x,
        const double y,
        const double hdg,
        const double length,
        const double curvature);

    void AddRoadGeometrySpiral(
        carla::road::Road *road,
        const double s,
        const double x,
        const double y,
        const double hdg,
        const double length,
        const double curvStart,
        const double curvEnd);

    void AddRoadGeometryPoly3(
        carla::road::Road *road,
        const double s,
        const double x,
        const double y,
        const double hdg,
        const double length,
        const double a,
        const double b,
        const double c,
        const double d);

    void AddRoadGeometryParamPoly3(
        carla::road::Road *road,
        const double s,
        const double x,
        const double y,
        const double hdg,
        const double length,
        const double aU,
        const double bU,
        const double cU,
        const double dU,
        const double aV,
        const double bV,
        const double cV,
        const double dV,
        const std::string p_range);

    // called from profiles parser
    void AddRoadElevationProfile(
        const int32_t road_id,
        const double s,
        const double a,
        const double b,
        const double c,
        const double d);

    void AddRoadLateralSuperelevation(
        const int32_t road_id,
        const double s,
        const double a,
        const double b,
        const double c,
        const double d);

    void AddRoadLateralCrossfall(
        const int32_t road_id,
        const double s,
        const double a,
        const double b,
        const double c,
        const double d,
        const std::string side);

    void AddRoadLateralShape(
        const int32_t road_id,
        const double s,
        const double a,
        const double b,
        const double c,
        const double d,
        const double t);

    // Signal methods
    void AddSignal(
        const uint32_t road_id,
        const uint32_t signal_id,
        const float s,
        const float t,
        const std::string name,
        const std::string dynamic,
        const std::string orientation,
        const float zOffset,
        const std::string country,
        const std::string type,
        const std::string subtype,
        const float value,
        const std::string unit,
        const float height,
        const float width,
        const std::string text,
        const float hOffset,
        const float pitch,
        const float roll);

    void AddValidityToLastAddedSignal(
        uint32_t road_id,
        uint32_t signal_id,
        int32_t from_lane,
        int32_t to_lane);

    // called from junction parser
    void AddJunction(const int32_t id, const std::string name);

    void AddConnection(
        const int32_t junction_id,
        const int32_t connection_id,
        const int32_t incoming_road,
        const int32_t connecting_road);

    void AddLaneLink(
        const int32_t junction_id,
        const int32_t connection_id,
        const int32_t from,
        const int32_t to);

    void AddRoadSection(
        const uint32_t road_id,
        const uint32_t section_index,
        const double s,
        const double a,
        const double b,
        const double c,
        const double d);

    void SetRoadLaneLink(
        const uint32_t road_id,
        const int32_t section_index,
        const int32_t lane_id,
        const std::string lane_type,
        const bool lane_level,
        const int32_t predecessor,
        const int32_t successor);

    // called from lane parser
    void CreateLaneAccess(
        const Lane* lane,
        const float s,
        const std::string restriction);

    void CreateLaneBorder(
        const Lane* lane,
        const float s,
        const float a,
        const float b,
        const float c,
        const float d);

    void CreateLaneHeight(
        const Lane* lane,
        const float s,
        const float inner,
        const float outer);

    void CreateLaneMaterial(
        const Lane* lane,
        const float s,
        const std::string surface,
        const float friction,
        const float roughness);

    void CreateLaneOffset(
        const Lane* lane,
        const float s,
        const float a,
        const float b,
        const float c,
        const float d);

    void CreateLaneRule(
        const Lane* lane,
        const float s,
        const std::string value);

    void CreateLaneVisibility(
        const Lane* lane,
        const float s,
        const float forward,
        const float back,
        const float left,
        const float right);

    void CreateLaneWidth(
        const Lane* lane,
        const float s,
        const float a,
        const float b,
        const float c,
        const float d);

    void CreateRoadMark(
        const Lane* lane,
        const int road_mark_id,
        const float s,
        const std::string type,
        const std::string weight,
        const std::string color,
        const std::string material,
        const float width,
        const std::string lane_change,
        const float height,
        const std::string type_name,
        const float type_width);

    void CreateRoadMarkTypeLine(
        const Lane* lane,
        const int road_mark_id,
        const float length,
        const float space,
        const float tOffset,
        const float s,
        const std::string rule,
        const float width);

    void CreateLaneSpeed(
        const Lane* lane,
        const float s,
        const float max,
        const std::string unit);

    void AddValidityToSignal(
        const uint32_t road_id,
        const uint32_t signal_id,
        const int32_t from_lane,
        const int32_t to_lane);

    void AddValidityToSignalReference(
        const uint32_t road_id,
        const uint32_t signal_reference_id,
        const int32_t from_lane,
        const int32_t to_lane);

    void AddSignalReference(
        const uint32_t road_id,
        const uint32_t signal_reference_id,
        const float s_position,
        const float t_position,
        const std::string signal_reference_orientation);

    void AddDependencyToSignal(
        const uint32_t road_id,
        const uint32_t signal_id,
        const uint32_t dependency_id,
        const std::string dependency_type);

    Road *GetRoad(
        const RoadId road_id
    );

    Lane *GetLane(
        const RoadId road_id,
        const LaneId lane_id,
        const float s);

  private:

    MapData _map_data;

    /// Set the total length of each road based on the geometries
    void SetTotalRoadSegmentLength();

    /// Create the pointers between RoadSegments based on the ids
    void CreatePointersBetweenRoadSegments();

  private:
    /// Map to temporary store all the lane infos until the map is built, so they
    /// can be added all together
    std::unordered_map<const Lane *, std::vector<std::unique_ptr<element::RoadInfo>>>
        _temp_lane_info_container;

    std::unordered_map<carla::road::Road *, std::vector<std::unique_ptr<carla::road::element::RoadInfo>>> _road_info;

  };

} // namespace road
} // namespace carla
