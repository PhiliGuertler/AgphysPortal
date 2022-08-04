#include "playerCreator.h"


Player::Player(ParticleSystem& particleSystem,int index) 
	: m_particleSystem(particleSystem)
	, bodyIndex(index)
	, position(vec3(0,5,0))
{}
Player::Player(const Player& x) 
	: m_particleSystem(x.m_particleSystem)
	, bodyIndex(x.bodyIndex)
{}

void Player::move(const Saiga::Ray& cameraRay, int direction){
	//m_particleSystem.movePlayerFromCamera(cameraRay,direction);	
}
