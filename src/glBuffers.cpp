#include "glBuffers.h"

#include "hacky.hack"

// ######################################################################### //
// ### GLBufferManager ##################################################### //
// ######################################################################### //

std::unique_ptr<GLBufferManager> GLBufferManager::s_instance = nullptr;

GLBufferManager& GLBufferManager::get() {
	if(s_instance == nullptr) s_instance = std::unique_ptr<GLBufferManager>(new GLBufferManager());
	return *s_instance;
}

void GLBufferManager::shutdown() {
	// deletes the singleton instance. This must happen before the glContext gets destroyed to avoid a segfault
	s_instance = nullptr;
}

void GLBufferManager::map() {
	// interop the particlePositionRadius gl-buffer
	m_interopPositions.map();
	void* ptrPositions = m_interopPositions.getDevicePtr();
	m_particleSystem->setParticlePositions(ptrPositions);

	// interop the particle's colors gl-buffer
	m_interopColors.map();
	void *ptrColors = m_interopColors.getDevicePtr();
	m_particleSystem->setParticleColors(ptrColors);
}

void GLBufferManager::unmap() {
	// unmap the particlePositionRadius gl-buffer
	m_interopPositions.unmap();

	// unmap the particle's colors gl-buffer
	m_interopColors.unmap();
}

void GLBufferManager::updateInteropReferences() {
	resetInteropReferences();
	// register the current buffer of the particlePositionRadius gl-buffer
	m_interopPositions.registerBuffer(m_positions.getBufferObject());

	// register the current particle's colors gl-buffer
	m_interopColors.registerBuffer(m_colors.getBufferObject());
}

void GLBufferManager::resetInteropReferences() {
	// unregister the current buffer of the particlePositionRadius gl-buffer
	m_interopPositions.unregisterBuffer();
	Saiga::CUDA::Interop *positionPtr = &m_interopPositions;
	cudaGraphicsResource **ptr = &(positionPtr->*result<InteropFix>::ptr);
	*ptr = nullptr;

	// unregister the current buffer of the particle's colors gl-buffer
	m_interopColors.unregisterBuffer();
	Saiga::CUDA::Interop *colorPtr = &m_interopColors;
	ptr = &(colorPtr->*result<InteropFix>::ptr);
	*ptr = nullptr;
}

void GLBufferManager::bind() {
	// bind the vertex array + vertex buffer for vertices
	m_positions.bind();
	// bind the color buffer and set it as input for the shader
	m_colors.bind();
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2,4,GL_FLOAT,GL_FALSE,sizeof(ParticleColor),NULL);
}

void GLBufferManager::unbind() {
	// unbind the vertex array which will automatically unbind all further buffers too
	m_positions.unbind();
}
