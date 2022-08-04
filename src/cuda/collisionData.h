#pragma once

// ######################################################################### //
// ### Collision Structs ################################################### //
// ######################################################################### //

/**
 *  Struct of two ints representing the indices of two colliding
 *  particles. Overrides operator() to be used as a predicate for remove_if.
 */
struct SAIGA_ALIGN(8) CollisionParticleData {
    int particleID1 = -1;
    int particleID2 = -1;
    
    __host__ __device__
    bool operator()(const CollisionParticleData x) {
        // return true if no collision is set, so that remove_if will
        // remove this
        return x.particleID1 == -1 || x.particleID2 == -1;
    }
};

/**
 *  Struct of two ints representing the indices of two colliding
 *  particles. Overrides operator() to be used as a predicate for .
 */
struct SAIGA_ALIGN(16) CollisionRayData {
    int particleID;
    vec3 rayOrigin; // currently unused
    float distance = INFINITY;
    float distanceToCenter = INFINITY;

    __host__ __device__
    bool operator()(const CollisionRayData& a, const CollisionRayData& b) {
        if(a.distance < 0.f && b.distance < 0.f) {
            return a.distanceToCenter < b.distanceToCenter;
        } else if(a.distance < 0.f) {
            return true;
        } else if(b.distance < 0.f) {
            return false;
        } else {
            return a.distanceToCenter < b.distanceToCenter;
        }
    }
};

/**
 *  Struct of two ints representing the indices of a particle and a plane
 *  that are colliding. Overrides operator() to be used as a predicate remove_if.
 */
struct SAIGA_ALIGN(8) CollisionPlaneData {
    int particleID = -1;
    int planeID = -1;

    __host__ __device__
    bool operator()(const CollisionPlaneData x) {
        // return true if this is not set
        return x.particleID == -1 && x.planeID == -1;
    }
};
struct SAIGA_ALIGN(8) CollisionWallData {
    int particleID = -1;
    int wallID = -1;

    __host__ __device__
    bool operator()(const CollisionWallData x) {
        // return true if this is not set
        return x.particleID == -1 && x.wallID == -1;
    }
};

// ######################################################################### //
// ### Triangle Intersection FUN ########################################### //
// ######################################################################### //

struct SAIGA_ALIGN(16) Tryeck {
	vec3 a;
	vec3 b;
	vec3 c;

	__host__ __device__
	inline vec3 normal() const {
		vec3 triangleNormal = (b - a).cross(c - a).normalized();
		return triangleNormal;
	}
};

struct SAIGA_ALIGN(8) CollisionTryeckData {
	int particleID = -1;
	int tryeckID = -1;

	__host__ __device__
	bool operator()(const CollisionTryeckData x) {
		return x.particleID == -1 && x.tryeckID == -1;
	}
};
