#pragma once
#include "bamboo/base/macro.h"

namespace welkin::bamboo {
struct BAMBOO_EXPORT LasPoint {
    double coordinates[3];
    unsigned char rgb[3];
};
}