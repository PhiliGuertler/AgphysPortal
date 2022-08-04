##GL_VERTEX_SHADER
#version 430 core

layout(location = 0) in vec4 in_position_radius;

out vec2 v_coord;

void main() {
	v_coord = 0.5 * in_position_radius.xy + 0.5;
	gl_Position = vec4(in_position_radius.xy, 0.0, 1.0);
}


##GL_FRAGMENT_SHADER
#version 430 core

in vec2 v_coord;

uniform sampler2D u_thickness;

out float o_intensity;

void main() {
	float p = texture(u_thickness, v_coord).x;

	if(p <= 1.0) {
		o_intensity = 0.0;
		return;
	}

	float pexp = pow(p, 1.25);
	o_intensity = pexp / (3 + pexp);
}

##end