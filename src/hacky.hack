#pragma once

#include "rendering/CustomDeferredRenderer.h"
#include "saiga/cuda/interop.h"

// ### HACKY HACKY ### //
template<typename Tag>
struct result {
  /* export it ... */
  typedef typename Tag::type type;
  static type ptr;
};

template<typename Tag>
typename result<Tag>::type result<Tag>::ptr;

template<typename Tag, typename Tag::type p>
struct rob : result<Tag> {
  /* fill it ... */
  struct filler {
    filler() { result<Tag>::ptr = p; }
  };
  static filler filler_obj;
};

template<typename Tag, typename Tag::type p>
typename rob<Tag, p>::filler rob<Tag, p>::filler_obj;

// access to private DefferedRenderer menbers
struct Rendererf {
	typedef GBuffer DeferredRenderer::*type;
};

template struct rob<Rendererf, &DeferredRenderer::gbuffer>;

struct InteropFix {
	typedef cudaGraphicsResource * Saiga::CUDA::Interop::*type;
};

template struct rob<InteropFix, &Saiga::CUDA::Interop::graphic_resource>;
// ### HACKY HACKY ### //


