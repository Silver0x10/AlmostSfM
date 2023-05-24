#pragma once 
#include <vector>
#include <map>
#include "utils.h"

namespace pr {

    void bundle_adjustment(vector<Camera>& cameras, map<int, Vec3f> landmarks);

}

