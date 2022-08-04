#include "Skybox.h"

#include "saiga/core/geometry/triangle_mesh_generator.h"

namespace Saiga
{
Skybox2::Skybox2()
{
    AABB bb(make_vec3(-1), make_vec3(1));
    auto sb = TriangleMeshGenerator::createSkyboxMesh(bb);
    mesh.fromMesh(*sb);
}

void Skybox2::setPosition(const vec3& p)
{
    model.col(3) = vec4(p[0], 0, p[2], 1);
}

void Skybox2::setDistance(float d)
{
    model(0, 0) = d;
    model(1, 1) = d;
    model(2, 2) = d;
}


void Skybox2::render(Camera* cam)
{
    shader->bind();
    shader->uploadModel(model);
    shader->uploadTexture(cube_texture.get());

    mesh.bindAndDraw();

    cube_texture->unbind();

    shader->unbind();
}

}  // namespace Saiga
