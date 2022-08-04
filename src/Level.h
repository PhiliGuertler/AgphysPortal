#pragma once

#include "AgphysCudaConfig.h"

#include "saiga/core/camera/camera.h"
#include "saiga/opengl/assets/simpleAssetObject.h"
#include "saiga/opengl/assets/objAssetLoader.h"
#include "saiga/opengl/assets/coloredAsset.h"

#include "cuda/collisionData.h"

namespace Controls {

	class Level {
		public:
			Level();
			Level(const std::string& filepath);
			virtual ~Level() = default;

			// if there is a texture for the level, use loadLevelFromFileTextured
			void loadLevelFromFileTextured(const std::string& filepath);
			// if there is no texture for the level, use loadLevelFromFileColored
			void loadLevelFromFileColored(const std::string& filepath);

			std::vector<Tryeck> getTryeckList() const;

			void setCenter(const vec3& center);
			void setScale(const vec3& scale);

			void render(Saiga::Camera *cam);
			void renderWireframe(Saiga::Camera *cam);
			void renderDepth(Saiga::Camera *cam);
		protected:
			Saiga::ObjAssetLoader m_loader;

			Saiga::SimpleAssetObject m_assetObject;
			std::shared_ptr<Saiga::Asset> m_asset;
			bool m_isTextured;
	};

}
