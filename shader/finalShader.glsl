##GL_VERTEX_SHADER
#version 400 core

layout(location = 0) in vec4 in_position_radius;

out vec2 v_coord;

void main() {
	v_coord = 0.5 * in_position_radius.xy; + 0.5;
	gl_Position = vec4(in_position_radius.xy, 0.0, 1.0);
}



##GL_FRAGMENT_SHADER
#version 400 core

in vec2 v_coord;

uniform sampler2D u_fluidMap;
uniform sampler2D u_foamIntensityMap;
uniform sampler2D u_foamRadianceMap;
uniform vec2 u_screenSize;

layout(location = 0) out vec4 o_fragColor;

const float PI = 3.14159265358979323846;

void main() {
	vec3 tmp = texture(u_foamRadianceMap, v_coord).xyz;
	float foamIntensity = texture(u_foamIntensityMap, v_coord).x;
	float foamRadiance = tmp.x;
	float hpass = tmp.y;
	float i = tmp.z;
	vec4 fluid = texture(u_fluidMap, v_coord).xyzw;

	float sum = 0;
	float totalWeight = 0;
	hpass *= 1.5;

	for(float x = -hpass; x < hpass; x += 1) {
		for(float y = -hpass; y < hpass; y += 1) {
			vec2 cxy = vec2(x / u_screenSize.x, y / u_screenSize.y);
			float weight = exp(-pow(length(vec2(x,y)), 2) / (pow(hpass, 2) * 2)) * (1 / (2*PI * pow(hpass, 2)));
			sum += texture(u_foamRadianceMap, v_coord + cxy).x * weight;
			totalWeight += weight;
		}
	}
	sum /= totalWeight;

	vec3 squiggly = clamp(sum * (vec3(1, 1, 1) - vec3(0, 0.2, 0.6)), 0, 1);
	o_fragColor = (1 - foamIntensity) * fluid + (foamIntensity * vec4((vec3(0.9) - squiggly), 1));
}

##end
