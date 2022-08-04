#pragma once

#include "saiga/cuda/interop.h"
#include "saiga/opengl/instancedBuffer.h"
#include "cuda/particleSystem/particleSystem.h"

/**
 *	Singleton class providing access to interop'd gl-buffer objects
 */
class GLBufferManager {
	public:
		/**
		 *	Singleton getter
		 */
		static GLBufferManager& get();

		/**
		 *	this should be called before the gl-context is destroyed to avoid a segfault
		 */
		static void shutdown();

	public:
		/**
		 *	default destructor
		 */
		~GLBufferManager() = default;

		/**
		 *	must be initialized as soon as a new particle system has been created.
		 */
		inline void setParticleSystem(std::shared_ptr<ParticleSystem> particleSystem) {
			m_particleSystem = particleSystem;
		}

		/**
		 *	maps interops to use gl buffers in CUDA-kernels
		 */
		void map();
		
		/**
		 *	unmaps interops to enable for example drawing of gl-buffers
		 */
		void unmap();

		/**
		 *	updates the referenced buffer-objects of the interops
		 */
		void updateInteropReferences();
		/**
		 *	unregisters gl-buffers of the interops
		 */
		void resetInteropReferences();
		
		void unregisterPositions();
		void unregisterColors();

		inline Saiga::VertexBuffer<ParticlePositionRadius>& getParticlePositions() {
			return m_positions;
		}

		inline Saiga::TemplatedBuffer<ParticleColor>& getParticleColors() {
			return m_colors;
		}

		/**
		 *	binds vertexbuffers for rendering of particle positions, radii and colors
		 */
		void bind();
		/**
		 *	unbinds vertexbuffers
		 */
		void unbind();

	private:
		GLBufferManager()
			: m_interopPositions()
			, m_positions()
			, m_interopColors()
			, m_colors(GL_ARRAY_BUFFER)
			, m_particleSystem(nullptr)
		{
			// empty
		}

		// singleton instance
		static std::unique_ptr<GLBufferManager> s_instance;

		// --- Particle Positions --- //
		Saiga::CUDA::Interop m_interopPositions;
		Saiga::VertexBuffer<ParticlePositionRadius> m_positions;

		// --- Particle Colors --- //
		Saiga::CUDA::Interop m_interopColors;
		Saiga::TemplatedBuffer<ParticleColor> m_colors;

		// reference to the current particle system
		std::shared_ptr<ParticleSystem> m_particleSystem;
};
