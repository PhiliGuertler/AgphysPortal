#include "saiga/opengl/indexedVertexBuffer.h"
#include "saiga/cuda/interop.h"
#include "saiga/cuda/cudaHelper.h"

#include "structures.h"


class CustomBuffer {
	public:
		// maps this classes vertexbuffer to be used with cuda
		WaterVertex *mapVertexBuffer();
		ArrayView<WaterVertex> getMappedVertexBuffer();
		void unmapVertexBuffer();
	
		// maps this classes indexbuffer to be used with cuda
		GLuint *mapIndexBuffer();
		ArrayView<GLuint> getMappedIndexBuffer();
		void unmapIndexBuffer();

		void bind() const;
		void unbind() const;

		void draw(int length = -1, int offset = 0);

		void bindAndDraw();

		void resize(int numVertices, int numIndices, GLenum usage = GL_DYNAMIC_DRAW);

		void set(Saiga::ArrayView<WaterVertex> vertices, Saiga::ArrayView<GLuint> indices, GLenum usage);

		void setOnDevice(const thrust::device_vector<WaterVertex>& vertices, const thrust::device_vector<GLuint>& indices, GLenum usage);

		inline void setDrawMode(GLenum mode) { m_vertexBuffer.setDrawMode(mode); }

		inline bool isEverythingAwesome() const {return m_everythingIsAwesome;}

	private:
		Saiga::CUDA::Interop m_vertexBufferInterop;
		Saiga::CUDA::Interop m_indexBufferInterop;

		Saiga::VertexBuffer<WaterVertex> m_vertexBuffer;
		Saiga::IndexBuffer<GLuint> m_indexBuffer;

		int m_numVertices = 0;
		int m_numIndices = 0;

		bool m_everythingIsAwesome = false;
};
