#pragma once

#include "particle.h"

// ######################################################################### //
// ### Predicate Structs to be used with thrust methods #################### //
// ######################################################################### //

/**
 *  A nifty hash function which is kind of quick for the linked cell structure
*/
__host__ __device__
inline int hashMe(vec3 position,  float cellLength, int arrayLength) {
    int xPart = floor(position.x() / cellLength);
    int yPart = floor(position.y() / cellLength);
    int zPart = floor(position.z() / cellLength);

    int hash = (512 * 512 * xPart + 512 * yPart + zPart) & (arrayLength -1);
    return hash;

}

/**
 *  Sorts by particle radius
 */
struct MaxRadius {
    __host__ __device__
    bool operator()(const ParticlePositionRadius& x, const ParticlePositionRadius& y) {
        return x.radius < y.radius;
    }
};

/**
 *	Sorts particles by their hash value
 */
struct SortParticle {

    float m_cellLength;
    int m_arrayLength;

    SortParticle(float cellLength, int arrayLength)
        : m_cellLength(cellLength)
        , m_arrayLength(arrayLength)
    {

    }

    __host__ __device__
    bool operator()(const ParticlePositionRadius& a, const ParticlePositionRadius& b) {
        return hashMe(a.position, m_cellLength, m_arrayLength) < hashMe(b.position, m_cellLength, m_arrayLength);
    }
};
