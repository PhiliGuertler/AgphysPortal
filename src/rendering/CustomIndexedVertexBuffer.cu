#include "saiga/cuda/cudaHelper.h"
#include "saiga/cuda/memory.h"

#include "CustomIndexedVertexBuffer.h"

#include "../hacky.hack"


WaterVertex *CustomBuffer::mapVertexBuffer() {
	// register and map the buffer using an interop object
	m_vertexBufferInterop.map();

	// return an arrayview of the mapped buffer
	void *ptr = m_vertexBufferInterop.getDevicePtr();
	return (WaterVertex *)ptr;
}

void CustomBuffer::unmapVertexBuffer() {
	// unmap the buffer
	m_vertexBufferInterop.unmap();
}

ArrayView<WaterVertex> CustomBuffer::getMappedVertexBuffer() {
	auto ptr = mapVertexBuffer();
	return ArrayView<WaterVertex>(ptr, m_numVertices);
}

GLuint *CustomBuffer::mapIndexBuffer() {
	// register and map the buffer using an interop object
	m_indexBufferInterop.map();

	// return an arrayview of the mapped buffer
	void *ptr = m_indexBufferInterop.getDevicePtr();
	return (GLuint *)ptr;
}

void CustomBuffer::unmapIndexBuffer() {
	// unmap the buffer
	m_indexBufferInterop.unmap();
}

ArrayView<GLuint> CustomBuffer::getMappedIndexBuffer() {
	auto ptr = mapIndexBuffer();
	return ArrayView<GLuint>(ptr, m_numIndices);
}

void CustomBuffer::bind() const {
	m_vertexBuffer.bind();
}

void CustomBuffer::unbind() const {
	m_vertexBuffer.unbind();
}

void CustomBuffer::draw(int length, int offset) {
	if(m_numIndices > 0 && m_numVertices > 0) {
		GLenum drawMode = m_vertexBuffer.getDrawMode();
		glDrawElements(drawMode
				, length < 0 ? m_numIndices : length
				, GL_UNSIGNED_INT
				, (void *)(offset * sizeof(GLuint)));
	}
}

void CustomBuffer::bindAndDraw() {
	bind();
	draw();
	unbind();
}

void CustomBuffer::resize(int numVertices, int numIndices, GLenum usage) {
	m_numIndices = numIndices;
	m_numVertices = numVertices;

	std::vector<WaterVertex> verts(m_numVertices);
	std::vector<GLuint> inds(m_numIndices);
	set(verts, inds, usage);

#if 0
	if(m_vertexBuffer.getElementCount() < numVertices) {
		// resize the vertex buffer
		std::vector<WaterVertex> initialData(numVertices);
		std::vector<GLuint> indexData;
		if(m_indexBuffer.getElementCount() != 0) {
			indexData.resize(m_indexBuffer.getElementCount());
		} else {
			// first resize ever: the index buffer needs some data to become a valid gl buffer
			indexData.resize(numIndices);
		}

		set(initialData, indexData, usage);
	}
	if(m_indexBuffer.getElementCount() < numIndices) {
		// resize the index buffer
		std::vector<WaterVertex> initialData(m_vertexBuffer.getElementCount());
		std::vector<GLuint> indexData(numIndices);

		set(initialData, indexData, usage);
	}
	m_numIndices = numIndices;
	m_numVertices = numVertices;
#endif
}

void CustomBuffer::set(Saiga::ArrayView<WaterVertex> vertices, Saiga::ArrayView<GLuint> indices, GLenum usage) {
	if(vertices.size() == 0 || indices.size() == 0) {
		m_everythingIsAwesome = false;
	} else {
		m_everythingIsAwesome = true;
	}

	m_vertexBufferInterop.unregisterBuffer();
	Saiga::CUDA::Interop *positionPtr = &m_vertexBufferInterop;
	cudaGraphicsResource **ptr = &(positionPtr->*result<InteropFix>::ptr);
	*ptr = nullptr;

	m_indexBufferInterop.unregisterBuffer();
	positionPtr = &m_indexBufferInterop;
	ptr = &(positionPtr->*result<InteropFix>::ptr);
	*ptr = nullptr;

	// crashes if called with an arrayview representing a device vector
	m_vertexBuffer.set(vertices, usage);
	m_indexBuffer.set(indices, usage);

	m_vertexBuffer.bind();
	m_indexBuffer.bind();
	m_vertexBuffer.unbind();
	m_indexBuffer.unbind();

	if(m_vertexBuffer.getBufferObject() != 0 && vertices.size() > 0) {
		m_vertexBufferInterop.registerBuffer(m_vertexBuffer.getBufferObject());
	}
	if(m_indexBuffer.getBufferObject() != 0 && indices.size() > 0) {
		m_indexBufferInterop.registerBuffer(m_indexBuffer.getBufferObject());
	}
}

void CustomBuffer::setOnDevice(const thrust::device_vector<WaterVertex>& vertices, const thrust::device_vector<GLuint>& indices, GLenum usage) {
	resize(m_numVertices, m_numIndices, usage);

	// map the buffers to be used with cuda
	auto verts = getMappedVertexBuffer();
	auto inds = getMappedIndexBuffer();

	// copy data using thrust::copy
	thrust::copy(vertices.begin(), vertices.end(), verts.device_begin());
	thrust::copy(indices.begin(), indices.end(), inds.device_begin());

	// unmap the buffers from cuda
	unmapVertexBuffer();
	unmapIndexBuffer();
}

template<>
void VertexBuffer<WaterVertex>::setVertexAttributes() {
	glEnableVertexAttribArray(0);	// enable in_position as 4 floats
	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 12 * sizeof(float), nullptr);

	glEnableVertexAttribArray(1);	// enable in_normal as 4 floats
	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 12 * sizeof(float), (void *)(4 * sizeof(float)));

	glEnableVertexAttribArray(2);	// enable in_vorticity as 1 float
	glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 12 * sizeof(float), (void *)(8 * sizeof(float)));
}
