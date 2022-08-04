##GL_VERTEX_SHADER
#version 400 core

layout(location = 0) in vec4 in_position_radius;

uniform mat4 u_modelView;
uniform mat4 u_proj;
uniform float u_pointRadius;
uniform float u_pointScale;

out vec3 v_pos;
out float v_lifetime;

void main() {
	float ri = u_pointRadius;
	float trl = in_position_radius.w;

	ri /= ((int(trl) % 5) + 1);

	vec4 viewPos = u_modelView * vec4(in_position_radius.xyz, 1.0);
	gl_Position = u_proj * viewPos;
	v_pos = viewPos.xyz;
	gl_PointSize = u_pointScale * (ri / gl_Position.w);

	v_lifetime = ri;
}


##GL_FRAGMENT_SHADER
#version 400 core

in vec3 v_pos;
in float v_lifetime;

uniform sampler2D u_foamDepthMap;
uniform sampler2D u_fluidDepthMap;
uniform mat4 u_modelView;
uniform mat4 u_proj;
uniform vec2 u_screenSize;
uniform float u_zNear;
uniform float u_zFar;

out float o_fThickness;

float linearizeDepth(float depth) {
	return (2.0 * u_zNear) / (u_zFar + u_zNear - depth * (u_zFar - u_zNear));
}

void main() {
	//calculate normal
	vec3 normal;
	normal.xy = gl_PointCoord * 2.0 - 1.0;
	float r2 = dot(normal.xy, normal.xy);
	
	if (r2 > 1.0f) {
		discard;
	}

	float r = sqrt(r2);
	if(r <= 1.0) {
		o_fThickness = 1.0 - pow(r, 1.5);
	} else {
		o_fThickness = 0.0;
		return;
	}

	o_fThickness *= pow(v_lifetime, 0.8);

	vec2 coord = gl_FragCoord.xy / u_screenSize.xy;

	float zFluid = texture(u_fluidDepthMap, coord).x;
	zFluid = linearizeDepth(zFluid);
	float zFoam = texture(u_foamDepthMap, coord).x;
	zFoam = linearizeDepth(zFoam);

	if(zFoam > zFluid) {
		if((zFoam - zFluid) * 10 <= 1.0) {
			o_fThickness += pow(1.0 - pow((zFoam - zFluid) * 10, 2), 4);
		} else {
			o_fThickness = 0.0;
		}
	}
}

##end
