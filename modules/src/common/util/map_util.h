/******************************************************************************
 * Copyright 2024 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

/**
 * @file
 * @brief Some map util functions.
 */

#pragma once

#include "google/protobuf/stubs/map_util.h"

/**
 * @namespace EDrive::common::util
 * @brief EDrive::common::util
 */
namespace EDrive {
namespace common {
namespace util {

// Expose some useful utils from protobuf.
// Find*()
using google::protobuf::FindOrDie;
using google::protobuf::FindOrDieNoPrint;
using google::protobuf::FindWithDefault;
using google::protobuf::FindOrNull;
using google::protobuf::FindPtrOrNull;
using google::protobuf::FindLinkedPtrOrNull;
using google::protobuf::FindLinkedPtrOrDie;
using google::protobuf::FindCopy;

// Contains*()
using google::protobuf::ContainsKey;
using google::protobuf::ContainsKeyValuePair;

// Insert*()
using google::protobuf::InsertOrUpdate;
using google::protobuf::InsertOrUpdateMany;
using google::protobuf::InsertAndDeleteExisting;
using google::protobuf::InsertIfNotPresent;
using google::protobuf::InsertOrDie;
using google::protobuf::InsertOrDieNoPrint;
using google::protobuf::InsertKeyOrDie;

// Lookup*()
using google::protobuf::LookupOrInsert;
using google::protobuf::AddTokenCounts;
using google::protobuf::LookupOrInsertNew;
using google::protobuf::LookupOrInsertNewLinkedPtr;
using google::protobuf::LookupOrInsertNewSharedPtr;

// Misc Utility Functions
using google::protobuf::UpdateReturnCopy;
using google::protobuf::InsertOrReturnExisting;
using google::protobuf::EraseKeyReturnValuePtr;
using google::protobuf::InsertKeysFromMap;
using google::protobuf::AppendKeysFromMap;
using google::protobuf::AppendValuesFromMap;

}  // namespace util
}  // namespace common
}  // namespace EDrive
