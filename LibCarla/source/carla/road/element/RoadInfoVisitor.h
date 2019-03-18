// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <iterator>
#include <memory>

namespace carla {
namespace road {
namespace element {

  class RoadElevationInfo;
  class RoadGeneralInfo;
  class RoadInfo;
  class RoadInfoLane;
  class RoadInfoLaneAccess;
  class RoadInfoLaneBorder;
  class RoadInfoLaneHeight;
  class RoadInfoLaneMaterial;
  class RoadInfoLaneOffset;
  class RoadInfoLaneRule;
  class RoadInfoLaneVisibility;
  class RoadInfoLaneWidth;
  class RoadInfoMarkRecord;
  class RoadInfoMarkTypeLine;
  class RoadInfoVelocity;
  class RoadInfoGeometry;

  class RoadInfoVisitor {
  public:

    virtual void Visit(RoadElevationInfo &) {}
    virtual void Visit(RoadGeneralInfo &) {}
    virtual void Visit(RoadInfoLane &) {}
    virtual void Visit(RoadInfoLaneAccess &) {}
    virtual void Visit(RoadInfoLaneBorder &) {}
    virtual void Visit(RoadInfoLaneHeight &) {}
    virtual void Visit(RoadInfoLaneMaterial &) {}
    virtual void Visit(RoadInfoLaneOffset &) {}
    virtual void Visit(RoadInfoLaneRule &) {}
    virtual void Visit(RoadInfoLaneVisibility &) {}
    virtual void Visit(RoadInfoLaneWidth &) {}
    virtual void Visit(RoadInfoMarkRecord &) {}
    virtual void Visit(RoadInfoMarkTypeLine &) {}
    virtual void Visit(RoadInfoVelocity &) {}
    virtual void Visit(RoadInfoGeometry &) {}
  };

  template <typename T, typename IT>
  class RoadInfoIterator : private RoadInfoVisitor {
  public:

    static_assert(std::is_same<std::unique_ptr<RoadInfo>, typename IT::value_type>::value, "Not compatible.");

    using value_type = T;
    using difference_type = typename IT::difference_type;
    using pointer = T *;
    using reference = T &;

    RoadInfoIterator(IT begin, IT end)
      : _it(begin),
        _end(end) {
      _success = false;
      for (; !IsAtEnd(); ++_it) {
        (*_it)->AcceptVisitor(*this);
        if (_success) {
          break;
        }
      }
    }

    RoadInfoIterator &operator++() {
      _success = false;
      while (!_success) {
        ++_it;
        if (IsAtEnd()) {
          break;
        }
        (*_it)->AcceptVisitor(*this);
      }
      return *this;
    }

    /// @todo to fix
    reference operator*() const {
      return static_cast<T &>(**_it);
    }

    pointer operator->() const {
      return static_cast<T *>(_it->get());
    }

    bool operator!=(const RoadInfoIterator &rhs) const {
      return _it != rhs._it;
    }

    bool operator==(const RoadInfoIterator &rhs) const {
      return !((*this) != rhs);
    }

    bool IsAtEnd() const {
      return _it == _end;
    }

  private:

    void Visit(T &) {
      _success = true;
    }

    IT _it;

    IT _end;

    bool _success;
  };

  template <typename T, typename Container>
  static auto MakeRoadInfoIterator(const Container &c) {
    auto begin = std::begin(c);
    auto end = std::end(c);
    return RoadInfoIterator<T, decltype(begin)>(begin, end);
  }

  template <typename T, typename IT>
  static auto MakeRoadInfoIterator(IT begin, IT end) {
    return RoadInfoIterator<T, decltype(begin)>(begin, end);
  }

} // namespace element
} // namespace road
} // namespace carla
