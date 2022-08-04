#include "WaterRenderer.h"


void WaterRenderer::updateMeshData(thrust::device_vector<WaterVertex> vertices, thrust::device_vector<GLuint> indices) {
	m_mesh.setData(vertices, indices);
}


// ######################################################################### //
// ### WaterMesh ########################################################### //

void WaterMesh::setData(thrust::device_vector<WaterVertex> vertices, thrust::device_vector<GLuint> indices) {
	m_buffer.setOnDevice(vertices, indices, GL_STATIC_DRAW);
}
