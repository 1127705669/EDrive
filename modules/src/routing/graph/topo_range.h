/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

namespace EDrive {
namespace routing {

class NodeSRange {
 public:
  static bool IsEnoughForChangeLane(double start_s, double end_s);
  static bool IsEnoughForChangeLane(double length);

 public:
  NodeSRange() = default;
  NodeSRange(double s1, double s2);
  virtual ~NodeSRange() = default;

  bool operator<(const NodeSRange& other) const;
  bool IsValid() const;
  double StartS() const;
  double EndS() const;
  bool IsEnoughForChangeLane() const;
  double Length() const;

  void SetStartS(double start_s);
  void SetEndS(double end_s);
  bool MergeRangeOverlap(const NodeSRange& other);

 private:
  double start_s_ = 0.0;
  double end_s_ = 0.0;
};

}  // namespace routing
}  // namespace EDrive
