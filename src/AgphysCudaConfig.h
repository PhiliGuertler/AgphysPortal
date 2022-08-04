#pragma once


#include "saiga/core/math/math.h"
#include "saiga/cuda/cudaHelper.h"

using namespace Saiga;

// using Saiga::mat3;
// using Saiga::mat4;
// using Saiga::quat;
// using Saiga::vec3;
// using Saiga::vec4;


#ifdef __CUDACC__

#else

using std::max;
using std::min;
#endif
