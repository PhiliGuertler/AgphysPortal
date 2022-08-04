#include "Level.h"

namespace Controls {

	Level::Level() 
		: m_loader()
		, m_assetObject()
		, m_asset(nullptr)
		, m_isTextured(false)
	{
		// empty
	}

	Level::Level(const std::string& filepath)
		: m_loader()
		, m_assetObject()
		, m_asset(nullptr)
		, m_isTextured(false)
	{
		loadLevelFromFileTextured(filepath);
	}

	void Level::loadLevelFromFileTextured(const std::string& filepath) {
		m_asset = m_loader.loadTexturedAsset(filepath);
		m_assetObject.asset = m_asset;
		m_isTextured = true;
	}

	void Level::loadLevelFromFileColored(const std::string& filepath) {
		m_asset = m_loader.loadBasicAsset(filepath);
		m_assetObject.asset = m_asset;
		m_isTextured = false;
	}

	std::vector<Tryeck> Level::getTryeckList() const {
		if(m_asset == nullptr) return {};
		std::vector<Saiga::Triangle> triangles;
		if(m_isTextured) {
			std::dynamic_pointer_cast<Saiga::TexturedAsset>(m_asset)->toTriangleList(triangles);
		} else {
			std::dynamic_pointer_cast<Saiga::ColoredAsset>(m_asset)->toTriangleList(triangles);
		}
		std::vector<Tryeck> result(triangles.size());
#pragma omp parallel for
		for(int i = 0; i < triangles.size(); ++i) {
			Saiga::Triangle& t = triangles[i];
			Tryeck tr;
			tr.a = t.a; tr.b = t.b; tr.c = t.c;
			result[i] = tr;
		}

		return result;
	}

	void Level::setCenter(const vec3& center) {
		m_assetObject.model(0,3) = center.x();
		m_assetObject.model(1,3) = center.y();
		m_assetObject.model(2,3) = center.z();
	}

	void Level::setScale(const vec3& scale) {
		m_assetObject.model(0,0) = scale.x();
		m_assetObject.model(1,1) = scale.y();
		m_assetObject.model(2,2) = scale.z();
	}

	void Level::render(Saiga::Camera *cam) {
		if(m_assetObject.asset != nullptr) {
			m_assetObject.render(cam);
		}
	}

	void Level::renderWireframe(Saiga::Camera *cam) {
		if(m_assetObject.asset != nullptr) {
			m_assetObject.renderWireframe(cam);
		}
	}

	void Level::renderDepth(Saiga::Camera *cam) {
		if(m_assetObject.asset != nullptr) {
			m_assetObject.renderDepth(cam);
		}
	}
	

}
