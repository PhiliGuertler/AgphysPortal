#pragma once

#include "saiga/opengl/indexedVertexBuffer.h"
#include "saiga/opengl/shader/basic_shaders.h"
#include "TextureCube.h"
#include "saiga/opengl/texture/Texture.h"
#include "saiga/opengl/vertex.h"

namespace Saiga
{
class Skybox2
{
   public:
    IndexedVertexBuffer<VertexNT, GLuint> mesh;
    std::shared_ptr<MVPTextureShader> shader;
    std::shared_ptr<Texture> texture;
    std::shared_ptr<TextureCube2> cube_texture;
    mat4 model = mat4::Identity();

    Skybox2();

    void setPosition(const vec3& p);
    void setDistance(float d);
    void render(Camera* cam);
};

}  // namespace Saiga
