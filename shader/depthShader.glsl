##GL_VERTEX_SHADER
#version 430

layout(location=0) in vec4 in_position_radius;

uniform mat4 u_modelView;
uniform mat4 u_proj;
uniform float u_pointScale;

out vec3 v_pos;
out float v_radius;

void main() {
	// compute position in view space
	vec4 viewPos = u_modelView * vec4(in_position_radius.xyz, 1.0);
	gl_Position = u_proj * viewPos;
	v_pos = viewPos.xyz;
	v_radius = in_position_radius.w;
	gl_PointSize = u_pointScale * (v_radius / gl_Position.w);
}

##GL_FRAGMENT_SHADER
#version 430

in vec3 v_pos;
in float v_radius;

uniform mat4 u_proj;

void main() {
	// compute screen space normal
	vec3 normal;
	normal.xy = gl_PointCoord * 2.0 - 1.0;
	float r2 = dot(normal.xy, normal.xy);

	if(r2 > 1.0) discard;

	normal.z = sqrt(1.0 - r2);

	// compute depth
	vec4 pixelPos = vec4(v_pos + normal * v_radius, 1.0);
	vec4 screenSpacePos = u_proj * pixelPos;

	gl_FragDepth = (screenSpacePos.z / screenSpacePos.w) * 0.5 + 0.5;
}

##end