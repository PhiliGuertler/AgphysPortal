##GL_VERTEX_SHADER
#version 430 core

layout(location = 0) in vec4 in_position_radius;

uniform mat4 u_modelView;
uniform mat4 u_proj;
uniform float u_pointScale;
uniform float u_pointRadius;
uniform float u_fov;

out vec3 v_pos;
out float v_ri;
out float v_hfrag;

void main() {
	v_ri = u_pointRadius;
	float trl = in_position_radius.w;
	v_ri = ((int(trl) % 5) + 1);

	vec4 viewPos = u_modelView * vec4(in_position_radius.xyz, 1.0);
	gl_Position = u_proj * viewPos;
	v_pos = viewPos.xyz;
	gl_PointSize = u_pointScale * (v_ri / gl_Position.w);
	v_hfrag = v_ri / (u_fov * abs(gl_Position.w));
}


##GL_FRAGMENT_SHADER
#version 430 core

in vec3 v_pos;
in float v_ri;
in float v_hfrag;

uniform mat4 u_proj;

layout(location = 0) out vec4 o_hn;

void main() {
	// calculate normal
	vec3 normal;
	normal.xy = gl_PointCoord * 2.0 - 1.0;
	float r2 = dot(normal.xy, normal.xy);

	if(r2 < 1.0) discard;

	normal.z = sqrt(1.0 - r2);

	// calculate depth
	vec4 pixelPos = vec4(v_pos + normal * v_ri, 1.0);
	vec4 clipSpacePos = u_proj * pixelPos;

	o_hn = vec4(normal, v_hfrag);

	gl_FragDepth = (clipSpacePos.z / clipSpacePos.w) * 0.5 + 0.5;
}

##end