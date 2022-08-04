##GL_VERTEX_SHADER

#version 430
layout(location = 0) in vec3 in_position;

out vec2 tc;

void main() {
	tc = vec2(in_position.x, in_position.y);
	tc = tc * 0.5f + 0.5f;
	gl_Position = vec4(in_position.x, in_position.y, 0, 1);
}


##GL_FRAGMENT_SHADER

#version 430
in vec2 tc;

uniform sampler2D texture;
uniform sampler2D depthTex;

uniform vec2 u_screenSize;

layout(location = 0) out vec4 out_color;

void main() {
	ivec2 tci = ivec2(tc * u_screenSize);
	vec4 tex = texelFetch(texture, tci, 0);

	float depth = texelFetch(depthTex, ivec2(tc), 0).r;

	gl_FragDepth = depth;
	out_color = tex;
}
