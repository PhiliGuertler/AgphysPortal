##GL_VERTEX_SHADER
#version 430

layout(location=0) in vec4 in_position_radius;

uniform mat4 u_modelView;
uniform mat4 u_proj;
uniform float u_pointScale;

void main() {
	// compute position in view space
	vec4 viewPos = u_modelView * vec4(in_position_radius.xyz, 1.0);
	gl_Position = u_proj * viewPos;
	gl_PointSize = u_pointScale * (in_position_radius.w / gl_Position.w);
}


##GL_FRAGMENT_SHADER
#version 430

out float thickness;

void main() {
	// calculate screen space normal
	vec3 normal;
	normal.xy = gl_PointCoord * 2.0 - 1.0;
	float r2 = dot(normal.xy, normal.xy);

	if(r2 > 1.0) discard;

	normal.z = sqrt(1.0 - r2);

	thickness = normal.z * 0.005f;
}
