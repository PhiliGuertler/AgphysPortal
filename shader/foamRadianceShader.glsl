##GL_VERTEX_SHADER
#version 400 core

layout(location = 0) in vec4 in_position_radius;

out vec2 v_coord;

void main() {
	v_coord = 0.5 * in_position_radius.xy + 0.5;
	gl_Position = vec4(in_position_radius.xy, 0.0, 1.0);
}



##GL_FRAGMENT_SHADER
#version 400 core

in vec2 v_coord;

uniform mat4 u_modelView;
uniform mat4 u_proj;
uniform mat4 u_inverseProj;
uniform sampler2D u_foamDepthMap;
uniform sampler2D u_fluidDepthMap;
uniform sampler2D u_foamNormalHMap;
uniform sampler2D u_foamIntensityMap;
uniform vec3 u_lightDirection;

layout(location = 0) out vec3 o_squiggly;

const float PI = 3.14159265358979323846f;

float rand(vec2 co) {
	return fract(sin(dot(co.xy, vec2(12.9898, 78.2333))) * 43758.5453);
}

void main() {
	float hfrag = texture(u_foamNormalHMap, v_coord).w;
	float foamDepth = texture(u_foamDepthMap, v_coord).x;
	float fluidDepth = texture(u_fluidDepthMap, v_coord).x;

	float omega = 0;
	float omegaBottom = 0;
	float hpass = 0;

	vec3 nfrag = texture(u_foamNormalHMap, v_coord).xyz;
	vec3 l = (u_modelView * vec4(u_lightDirection, 0.0)).xyz;

	float irr = abs(dot(l, nfrag));

	vec4 eyeCoord = vec4(((vec3(v_coord, foamDepth) * 2.0) - 1.0), 1.0);
	eyeCoord = u_inverseProj * eyeCoord;
	eyeCoord /= eyeCoord.w;

	for(float p = 0; p < 1; p += 1) {
		hpass = hfrag * (1.0 + 7.0 * p);
		float v = clamp(0.75 * PI * pow(hpass, 3) * 0.5, 16, 256);

		for(float i = 0; i < v; i += 1.0) {
			vec3 s = vec3(rand(vec2(10*v, 10*v)), rand(vec2(20*v,20*v)), rand(vec2(30*v, 30*v)));
			s = (s * 2.0) - 1.0;
			if(length(s) > 1) continue;

			float lambda = pow(1.0 - length(s), 2);
			s+= eyeCoord.xyz;
			vec4 sproj = u_proj * vec4(s, 1);
			sproj /= sproj.w;
			sproj = sproj * 0.5 + 0.5;

			float sampleFoamDepth = texture(u_foamDepthMap, sproj.xy).x;
			float sampleFluidDepth = texture(u_fluidDepthMap, sproj.xy).x;
			float sampleIntensity = texture(u_foamIntensityMap, sproj.xy).x;

			float delta = pow(max(1 - (abs(sampleFoamDepth - sproj.z) / 5), 0), 2);

			float k = ((sproj.z > sampleFoamDepth || sproj.z > sampleFluidDepth) && (delta > 0.0 && delta < 1.0)) ? 1.0 : 0.0;

			omega += lambda * delta * k * sampleIntensity;
			omegaBottom += lambda;
		}
	}

	omega /= omegaBottom;
	o_squiggly.x = clamp(pow(omega, 1.5) - 0.05, 0, 1);
	o_squiggly.y = hpass;
	o_squiggly.z = irr;
}

##end
