/******************************************************************************
 * Copyright 2022 The EDrive Authors. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <string>

namespace EDrive {
namespace EROS {

enum
{

}E;

class Status {

    explicit Status() {}
    
    ~Status() = default;

    static Status OK() { return Status(); }

    bool ok() const { return code_ == ErrorCode::OK; }

};

} // EROS
} // EDrive