#pragma once

#include "saiga/opengl/texture/TextureBase.h"

namespace Saiga
{
/*
 *  From Stackoverflow:
 *
 * Cube Maps have been specified to follow the RenderMan specification (for whatever reason),
 *  and RenderMan assumes the images' origin being in the upper left, contrary to the usual
 * OpenGL behaviour of having the image origin in the lower left. That's why things get swapped
 *  in the Y direction. It totally breaks with the usual OpenGL semantics and doesn't make
 * sense at all. But now we're stuck with it.
 *
 * -> Swap Y before creating a cube texture from a image
 */
class SAIGA_OPENGL_API TextureCube2 : public TextureBase
{
   public:
    TextureCube2() : TextureBase(GL_TEXTURE_CUBE_MAP) {}
    virtual ~TextureCube2() {}


    void setDefaultParameters();
    void uploadData(const void* data);

    void uploadData(GLenum target, const void* data);


    bool fromImage(std::vector<Image>& images);


    void create(int width, int height, GLenum color_type, GLenum internal_format, GLenum data_type,
                const void* data = nullptr);

   private:
    int width, height;
};

}  // namespace Saiga
