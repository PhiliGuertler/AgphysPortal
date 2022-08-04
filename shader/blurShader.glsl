##GL_VERTEX_SHADER
#version 430

layout(location = 0) in vec4 in_position_radius;

out vec2 v_coord;

void main() {
	v_coord = 0.5 * in_position_radius.xy + 0.5;
	gl_Position = vec4(in_position_radius.xy, 0.0, 1.0);
}


##GL_FRAGMENT_SHADER
#version 430

in vec2 v_coord;

uniform sampler2D u_depthMap;
uniform mat4 u_proj;
uniform vec2 u_blurDirection;
uniform float u_filterRadius;
uniform float u_blurScale;
uniform float u_blurDepthFalloff;

void main() {
	float depth = texture(u_depthMap, v_coord).x;

	if(depth <= 0.0) {
		gl_FragDepth = 0;
		return;
	}

	if(depth >= 1.0) {
		gl_FragDepth = depth;
		return;
	}

	float sum = 0.0;
	float wsum = 0.0;

	for(float x = -u_filterRadius; x < u_filterRadius; x += 1.0) {
		float s = texture(u_depthMap, v_coord + x * u_blurDirection).x;

		if(s >= 1.0) continue;

		float r = x * u_blurScale;
		float w = exp(-r * r);

		float r2 = (s - depth) * u_blurDepthFalloff;
		float g = exp(-r2*r2);

		sum += s * w * g;
		wsum += wsum;
	}

	if(wsum > 0.0) {
		sum /= wsum;
	}

	gl_FragDepth = sum;
}

##end