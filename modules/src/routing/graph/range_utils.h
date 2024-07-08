/******************************************************************************
  * Copyright 2024 The EDrive Authors. All Rights Reserved.
  *****************************************************************************/

#pragma once

#include <vector>

namespace EDrive {
namespace routing {

template <typename T>
int BinarySearchForSLarger(const std::vector<T>& sorted_vec, double value_s) {
  if (sorted_vec.empty()) {
    return -1;
  }
  int start_index = 0;
  int end_index = sorted_vec.size() - 1;
  double internal_s = 0.0;
  int middle_index = 0;
  while (end_index - start_index > 1) {
    middle_index = (start_index + end_index) / 2;
    internal_s = sorted_vec[middle_index].StartS();
    if (internal_s > value_s) {
      end_index = middle_index;
    } else {
      start_index = middle_index;
    }
  }
  double end_s = sorted_vec[start_index].EndS();
  if (value_s <= end_s) {
    return start_index;
  }
  return end_index;
}

template <typename T>
int BinarySearchForSSmaller(const std::vector<T>& sorted_vec, double value_s) {
  if (sorted_vec.empty()) {
    return -1;
  }
  int start_index = 0;
  int end_index = sorted_vec.size() - 1;
  double internal_s = 0.0;
  int middle_index = 0;
  while (end_index - start_index > 1) {
    middle_index = (start_index + end_index) / 2;
    internal_s = sorted_vec[middle_index].EndS();
    if (internal_s > value_s) {
      end_index = middle_index;
    } else {
      start_index = middle_index;
    }
  }
  double start_s = sorted_vec[end_index].StartS();
  if (value_s > start_s) {
    return end_index;
  }
  return start_index;
}

template <typename T>
int BinarySearchCheckValidSIndex(const std::vector<T>& sorted_vec, int index,
                                 double value_s) {
  if (index == -1) {
    return -1;
  }
  double start_s = sorted_vec[index].StartS();
  double end_s = sorted_vec[index].EndS();

  if (start_s <= value_s && end_s >= value_s) {
    return index;
  }
  return -1;
}

template <typename T>
int BinarySearchForStartS(const std::vector<T>& sorted_vec, double value_s) {
  int index = BinarySearchForSLarger(sorted_vec, value_s);
  return BinarySearchCheckValidSIndex(sorted_vec, index, value_s);
}

template <typename T>
int BinarySearchForEndS(const std::vector<T>& sorted_vec, double value_s) {
  int index = BinarySearchForSSmaller(sorted_vec, value_s);
  return BinarySearchCheckValidSIndex(sorted_vec, index, value_s);
}

}  // namespace routing
}  // namespace EDrive
