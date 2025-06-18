#pragma once

#include <QtGlobal>
#include <common/base/platform.h>


#ifdef BAMBOO_LIB
#define BAMBOO_EXPORT WELKIN_EXPORT
#else
#define BAMBOO_EXPORT WELKIN_IMPORT
#endif
