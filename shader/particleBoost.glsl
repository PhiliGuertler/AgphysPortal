##GL_VERTEX_SHADER



#version 400
layout(location=0) in vec4 in_position_radius;
layout(location=2) in vec4 in_color;

layout(location=3) in mat4 in_modelInstanced;

#include "camera.glsl"
uniform mat4 model;

out vec4 color;
out float radius;

void main()
{
    vec3 position = in_position_radius.xyz;
    float r = in_position_radius.w;
    gl_Position = model * vec4(position,1);
    color = vec4(in_color);
    radius = r;
}


##GL_GEOMETRY_SHADER
#version 400

layout(points) in;
in vec4[1] color;
in float[1] radius;
layout(triangle_strip, max_vertices=4) out;

#include "camera.glsl"
uniform mat4 model;

out vec2 tc;
out vec4 color2;
out vec4 centerV;
out float r;
out vec3 dir;
out vec4 posW;
out vec4 posV;


void main() {
    // if (color.a != 0.0) {
    //     return;
    // }
    if (color[0].a == 0.0) {
        return;
    }
    //create a billboard with the given radius
    vec4 centerWorld = gl_in[0].gl_Position;


    vec3 eyePos = -view[3].xyz * mat3(view);

    vec3 up = vec3(0,1,0);
    dir = normalize(eyePos-vec3(centerWorld));
    vec3 right = normalize(cross(up,dir));
    up = normalize(cross(dir,right));

    centerV = view*centerWorld;
    r = radius[0];
    color2 = color[0];

    float dx=radius[0];
    float dy=radius[0];

    vec4 ix=vec4(-1,1,-1,1);
    vec4 iy=vec4(-1,-1,1,1);
    vec4 tx=vec4(0,1,0,1);
    vec4 ty=vec4(0,0,1,1);


    for(int i =0; i<4;i++){
        tc.x = tx[i];
        tc.y = ty[i];
        posW = vec4(ix[i]*dx * right + iy[i]*dy * up,0) + centerWorld;
        posV = view * posW;
        gl_Position = proj*posV;
        EmitVertex();
    }
}


##GL_FRAGMENT_SHADER

#version 400


in float r;
in vec4 color2;
in vec2 tc;
in vec4 centerV;
in vec4 posW;
in vec4 posV;
in vec3 dir;

#include "camera.glsl"
uniform mat4 model;


#include "geometry/geometry_helper_fs.glsl"



void main() {
    vec2 reltc = tc*2-vec2(1);
    reltc *= r;
    float lensqr = dot(reltc, reltc);
    if(lensqr > r*r)
        discard;

    //solving x^2 + y^2 + z^2 = r^2 for z
    float z = sqrt(r*r - lensqr);

    vec4 vertexMV = posV;
    vertexMV.z += z;

    vec3 n = normalize(vec3(vertexMV)-vec3(centerV));
    //- vec4(dir*2.0f*(lensqr+0.05f),0);
    vec4 fragPosP = proj * vertexMV;
    fragPosP /= fragPosP.w;

#ifdef WRITE_DEPTH
    gl_FragDepth = fragPosP.z * 0.5f + 0.5f;
#endif

    vec3 data = vec3(1,0,0);
    setGbufferData(vec3(color2),n,vec4(data.xy,0,0));
}

##end
