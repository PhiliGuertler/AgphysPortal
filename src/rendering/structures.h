#pragma once

#include "../AgphysCudaConfig.h"

// ######################################################################### //
// ### Water Vertex ######################################################## //

struct WaterVertex {
	vec4 position = vec4(NAN, NAN, NAN, NAN);
	vec4 normal = vec4(NAN, NAN, NAN, NAN);
	vec4 vorticity = vec4(NAN, NAN, NAN, NAN);
};


