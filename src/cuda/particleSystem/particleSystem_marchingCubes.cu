#include "particleSystem.h"

#include "saiga/cuda/thread_info.h"	// Saiga::CUDA::ThreadInfo
#include "saiga/cuda/memory.h"		// Saiga::CUDA::vectorCopy

#include <thrust/extrema.h>
#include <thrust/transform_scan.h>

#include <saiga/opengl/shader/shaderLoader.h>

#include <iomanip>
#include <unordered_map>
#include <functional>

#include "particleSystem_tables.inl.cu"

#include "../../profiling/Profiler.h"
#include "../../rendering/WaterRenderer.h"

#define PRINT_VEC(a) a.x(), a.y(), a.z()
#define PRINT_VEC2(a) a.x(), a.y()
#define FUCCC "[%6.2f,%6.2f,%6.2f]"
#define FUCC "[%6.2f,%6.2f]"

#define MAX_VOXELS 12'000'000
#define MAX_FLUIDS 300'000

// ######################################################################### //
// ### Struct Predicates ################################################### //
// ######################################################################### //

struct ValidateTriangle {
	__host__ __device__
		inline bool operator()(const Tri& tri) {
			return tri.a.x() != tri.a.x();
		}
};

struct CompareVec3 {
	__host__ __device__
		inline float cropToLength(float in, int decimalPlaces) {
			int helper = pow(10, decimalPlaces);
			int cast = in * helper;
			float back = ((float)cast) / ((float)helper);
			return back;
		}

	__host__ __device__
		inline bool operator()(const vec3& a, const vec3& b) {
			return a.x()+0.001f < b.x() || a.y()+0.001f < b.y() || a.z()+0.001f < b.z();
		}
};

struct KeyVec3 {
	vec3 key;

	__host__ __device__
		bool operator==(const KeyVec3& other) const {
			vec3 diff = other.key - key;
			return diff.norm() < 0.001f;
		}
};

namespace std {
	template<>
		struct hash<KeyVec3> {
			std::size_t operator()(const KeyVec3& k) const {
				return std::hash<int>()((int)(k.key.x() * 10000))
					^ std::hash<int>()((int)(k.key.y() * 10000))
					^ std::hash<int>()((int)(k.key.z() * 10000));
			}
		};
}

struct TriangleCounterUnary {
	__host__ __device__
		int operator()(Tri x) const {
			int isTriangle = ValidateTriangle()(x) ? 0 : 1;
			return isTriangle;
		}
};

// ######################################################################### //
// ### Converter functions ################################################# //
// ######################################################################### //

/**
 *	returns an ivec3 with the amount of voxels in each ordinal axis
 */
static __host__ __device__ inline ivec3 computeNumVoxels(vec3 boundingBoxMin, vec3 boundingBoxMax, float gridSize) {
	vec3 bb = boundingBoxMax - boundingBoxMin;
	ivec3 result;
	result.x() = ceil(bb.x() / gridSize);
	result.y() = ceil(bb.y() / gridSize);
	result.z() = ceil(bb.z() / gridSize);

	return result;
}

/**
 *	turns a linear index into the voxel grid into a three dimensional index
 */
static __host__ __device__ inline ivec3 linearTo3DIndex(int index, vec3 boundingBoxMin, vec3 boundingBoxMax, float gridSize) {
	ivec3 numVoxels = computeNumVoxels(boundingBoxMin, boundingBoxMax, gridSize);

	// x is the fastest index, then y then z
	ivec3 result;
	result.z() = index / (numVoxels.x()*numVoxels.y());
	result.y() = (index / numVoxels.x()) % numVoxels.y();
	result.x() = index % numVoxels.x();

	return result;
}

/**
 *	linearizes a 3D index into the voxel grid
 */
static __host__ __device__ inline int index3DToLinearBigger(ivec3 index3D, vec3 boundingBoxMin, vec3 boundingBoxMax, float gridSize, ivec3 bigger) {
	ivec3 numVoxels = computeNumVoxels(boundingBoxMin, boundingBoxMax, gridSize);
	numVoxels += bigger;

	// x is the fastest index, then y then z
	int index = 0;
	index += index3D.x();
	index += index3D.y() * numVoxels.x();
	index += index3D.z() * numVoxels.x() * numVoxels.y();

	if(index < 0 || index >= numVoxels.x()*numVoxels.y()*numVoxels.z()) index = -1;

	return index;
}

static __host__ __device__ inline ivec3 linearTo3DIndexBigger(int index, vec3 boundingBoxMin, vec3 boundingBoxMax, float gridSize, ivec3 bigger) {
	ivec3 numVoxels = computeNumVoxels(boundingBoxMin, boundingBoxMax, gridSize);
	numVoxels += bigger;

	ivec3 trololo = linearTo3DIndex(index, boundingBoxMin, boundingBoxMax, gridSize);
	index = index3DToLinearBigger(trololo, boundingBoxMin, boundingBoxMax, gridSize, bigger);

	// x is the fastest index, then y then z
	ivec3 result;
	result.z() = index / (numVoxels.x()*numVoxels.y());
	result.y() = (index / numVoxels.x()) % numVoxels.y();
	result.x() = index % numVoxels.x();

	return result;
}
/**
 *	linearizes a 3D index into the voxel grid
 */
static __host__ __device__ inline int index3DToLinear(ivec3 index3D, vec3 boundingBoxMin, vec3 boundingBoxMax, float gridSize) {
	ivec3 numVoxels = computeNumVoxels(boundingBoxMin, boundingBoxMax, gridSize);

	// x is the fastest index, then y then z
	int index = 0;
	index += index3D.x();
	index += index3D.y() * numVoxels.x();
	index += index3D.z() * numVoxels.x() * numVoxels.y();

	//if(index < 0 || index >= numVoxels.x()*numVoxels.y()*numVoxels.z()) index = -1;

	return index;
}

/**
 *	computes the linearized voxel index from a position that should be inside the bounding box
 */
static __host__ __device__ inline int positionToVoxelIndex(vec3 position, vec3 boundingBoxMin, vec3 boundingBoxMax, float gridSize) {
	// map boundingBoxMin to [0,0,0]
	vec3 relativePosition = position - boundingBoxMin;
	// compute ordinal indices of this relative position
	int x = floor(relativePosition.x() / gridSize);
	int y = floor(relativePosition.y() / gridSize);
	int z = floor(relativePosition.z() / gridSize);

	ivec3 index3D = ivec3(x, y, z);
	return index3DToLinear(index3D, boundingBoxMin, boundingBoxMax, gridSize);
}

static __host__ __device__ inline vec3 index3DToVoxelCenter(ivec3 index3D, vec3 boundingBoxMin, float gridSize) {
	// start at the origin of the bounding box
	vec3 minCorner = boundingBoxMin;
	// move to the min corner of this voxel by adding the 3d-index times the grid size
	minCorner += vec3((float)index3D.x(), (float)index3D.y(), (float)index3D.z()) * gridSize;
	// add half the gridSize in each ordinal direction to get the center
	vec3 center = minCorner + vec3(gridSize, gridSize, gridSize) * 0.5f;
	return center;
}

/**
 *	computes the center of a voxel cell from a linear index
 */
static __host__ __device__ inline vec3 indexToVoxelCenter(int index, vec3 boundingBoxMin, vec3 boundingBoxMax, float gridSize) {
	ivec3 index3D = linearTo3DIndex(index, boundingBoxMin, boundingBoxMax, gridSize);

	return index3DToVoxelCenter(index3D, boundingBoxMin, gridSize);
}

// ######################################################################### //
// ### Surface Functions (to be taken from a paper) ######################## //
// ######################################################################### //

static __device__ inline float kernelPoly6(vec3 r, float h, float hToTheNegativeNinth) {
	float factor = (315.f / (64.f * M_PI)) * hToTheNegativeNinth;
	float result = 0.f;
	if(r.dot(r) < h*h) {
		float rip = h*h - r.dot(r);
		result = factor * rip * rip * rip;
	}
	return result;
}

// in the paper: "W"
static __device__ float smoothingKernel(vec3 r, float h, float sigma) {
	float result = sigma / (h*h*h);
	result *= kernelPoly6(r/h, 1.f, 1.f);
	return result;
}

/**
 *	computes the value of the implicitly defined surface of the fluid particles
 *	at position.
 *	Please do not call this function with positions outside of the boundingbox...
 */
static __device__ vec2 surfaceFunction(vec3 position
		, float h
		, float sigma
		, ArrayView<ParticlePositionRadius> positions
		, ArrayView<ParticleMomentumMass> masses
		, ArrayView<FluidVorticity> vorticities
		, ArrayView<int> gridBuckets
		, ArrayView<int> gridOverflow
		, vec3 boundingBoxMin
		, vec3 boundingBoxMax
		, float fluidsBegin
		, float gridSize) {
	float funcValue = 0.f;
	float vortemord = 0.f;

	// compute the grid index from the position
	int gridIndex = positionToVoxelIndex(position, boundingBoxMin, boundingBoxMax, gridSize);

	// return -1 if the index is out of bounds
	if(gridIndex >= gridBuckets.size() || gridIndex < 0) return vec2(-1.f, 0);

	// iterate over all particles in this cell
	int counter = 0;
	for(int neighborID = gridBuckets[gridIndex]; neighborID != -1; neighborID = gridOverflow[neighborID]) {
		// particleID is the neighborID plus fluidsBegin, as gridOverflow is only as big as there are numbers of fluid particles
		int particleID = neighborID+fluidsBegin;

		ParticlePositionRadius neighborPos;
		Saiga::CUDA::vectorCopy(positions.data()+particleID, &neighborPos);

		vec3 r = position - neighborPos.position;
		float colonel = smoothingKernel(r, h, sigma);

		// load neighbor mass
		ParticleMomentumMass mass;
		Saiga::CUDA::vectorCopy(masses.data()+particleID, &mass);

		// load neighbor density
		FluidVorticity density;
		Saiga::CUDA::vectorCopy(vorticities.data()+particleID, &density);

		vortemord += density.voriticity.norm();

		float tmp = mass.massinv * density.density;
		if(abs(tmp) > 0.f) {
			colonel *= 1.f/tmp;
		}

		funcValue += colonel;
		++counter;
	}
	if(counter == 0) {
		vortemord = 0.f;
	} else {
		vortemord /= (float)counter;
	}

	return vec2(funcValue, vortemord);
}

// ######################################################################### //
// ### Marching Cubes Kernels ############################################## //
// ######################################################################### //

__global__ void initField(ArrayView<vec2> voxelValues) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= voxelValues.size()) return;

	// surface value
	voxelValues[ti.thread_id].x() = 0.f;
	// vorticity value
	voxelValues[ti.thread_id].y() = 0.f;
}

static __device__ vec2 gridLookup(int index, ArrayView<vec2> gridValues) {
	if(index < 0 || index >= gridValues.size()) {
		return vec2(-1.f, 0);
	} else {
		return gridValues[index];
	}
}

struct CornerData {
	vec3 cornerPositions[8];
	vec2 cornerValues[8];
};

static __device__ CornerData createCornerData(int voxelIndex, float gridSize, vec3 bbMin, vec3 bbMax, ArrayView<vec2> gridValues) {
	ivec3 index3D = linearTo3DIndex(voxelIndex, bbMin, bbMax, gridSize);

	CornerData result;
	// get corner values
	result.cornerValues[0] = gridLookup(index3DToLinear(index3D+ivec3(0,0,0), bbMin, bbMax, gridSize), gridValues);
	result.cornerValues[1] = gridLookup(index3DToLinear(index3D+ivec3(1,0,0), bbMin, bbMax, gridSize), gridValues);
	result.cornerValues[2] = gridLookup(index3DToLinear(index3D+ivec3(0,1,0), bbMin, bbMax, gridSize), gridValues);
	result.cornerValues[3] = gridLookup(index3DToLinear(index3D+ivec3(1,1,0), bbMin, bbMax, gridSize), gridValues);

	result.cornerValues[4] = gridLookup(index3DToLinear(index3D+ivec3(0,0,1), bbMin, bbMax, gridSize), gridValues);
	result.cornerValues[5] = gridLookup(index3DToLinear(index3D+ivec3(1,0,1), bbMin, bbMax, gridSize), gridValues);
	result.cornerValues[6] = gridLookup(index3DToLinear(index3D+ivec3(0,1,1), bbMin, bbMax, gridSize), gridValues);
	result.cornerValues[7] = gridLookup(index3DToLinear(index3D+ivec3(1,1,1), bbMin, bbMax, gridSize), gridValues);


	// get corner positions
	result.cornerPositions[0] = index3DToVoxelCenter(index3D+ivec3(0,0,0), bbMin, gridSize);
	result.cornerPositions[1] = index3DToVoxelCenter(index3D+ivec3(1,0,0), bbMin, gridSize);
	result.cornerPositions[2] = index3DToVoxelCenter(index3D+ivec3(0,1,0), bbMin, gridSize);
	result.cornerPositions[3] = index3DToVoxelCenter(index3D+ivec3(1,1,0), bbMin, gridSize);

	result.cornerPositions[4] = index3DToVoxelCenter(index3D+ivec3(0,0,1), bbMin, gridSize);
	result.cornerPositions[5] = index3DToVoxelCenter(index3D+ivec3(1,0,1), bbMin, gridSize);
	result.cornerPositions[6] = index3DToVoxelCenter(index3D+ivec3(0,1,1), bbMin, gridSize);
	result.cornerPositions[7] = index3DToVoxelCenter(index3D+ivec3(1,1,1), bbMin, gridSize);

	return result;
}

// started once for each marching cube voxel
__global__ void marchingCubesKernel(ArrayView<vec2> gridValues, ArrayView<Tri> triangles, ArrayView<int> validTriangles, float gridSize, vec3 boundingBoxMin, vec3 boundingBoxMax, float isoValue) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= gridValues.size()) return;

	const float EPSTEIN = 0.001f;

	auto cornerData = createCornerData(ti.thread_id, gridSize, boundingBoxMin, boundingBoxMax, gridValues);

	unsigned int lookupIndex = 0;
	for(int i = 0; i < 8; ++i) {
		lookupIndex |= ((cornerData.cornerValues[i].x() > isoValue) ? 1 : 0) << i;
	}
	if(lookupIndex > 255) {
		printf("Lookup index is wrong: %d\n", lookupIndex);
	}

	// intersect edges
	vec3 vertices[12];
	float vorticities[12];
	for(int e = 0; e < 12; ++e) {
		if((1 << e) & e_pattern[lookupIndex]) {
			// look up some indíces
			int *lineData = edgeTable[e];
			int v0 = lineData[0];
			int v1 = lineData[1];

			// compute interpolation factor for this edge
			float part = cornerData.cornerValues[v1].x() - cornerData.cornerValues[v0].x();
			float u = 0.f;
			if(abs(part) >= EPSTEIN) {
				u = (isoValue - cornerData.cornerValues[v0].x()) / part;
			}

			vertices[e] = cornerData.cornerPositions[v0] + u * (cornerData.cornerPositions[v1] - cornerData.cornerPositions[v0]);
			vorticities[e] = cornerData.cornerValues[v0].y() + u * (cornerData.cornerValues[v1].y() - cornerData.cornerPositions[v0].y());
		} else {
			vertices[e] = vec3(NAN, NAN, NAN);
			vorticities[e] = NAN;
		}
	}

	auto triangleValues = triangleTable[lookupIndex];
	int t;
	for(t = 0; t < 5; ++t) {
		int lineIndex = triangleValues[t*3];
		if(lineIndex == -1) break;

		int triIndex = ti.thread_id*5+t;
		Tri tri = triangles[triIndex];

		validTriangles[triIndex] = 1;

		tri.a = vertices[triangleValues[t*3]];
		tri.b = vertices[triangleValues[t*3+1]];
		tri.c = vertices[triangleValues[t*3+2]];

		tri.aVorticity = vorticities[triangleValues[t*3]];
		tri.bVorticity = vorticities[triangleValues[t*3+1]];
		tri.cVorticity = vorticities[triangleValues[t*3+2]];


		// check for large triangles
		float maxSideLength = (tri.a - tri.b).norm();
		maxSideLength = max((tri.b - tri.c).norm(), maxSideLength);
		maxSideLength = max((tri.c - tri.a).norm(), maxSideLength);
		if(maxSideLength > gridSize * 1.01f * sqrt(3)) {
			tri = Tri();
			validTriangles[triIndex] = 0;
			float ab = (tri.a - tri.b).norm();
			float bc = (tri.b - tri.c).norm();
			float ca = (tri.c - tri.a).norm();
			printf("%8d Triangle is too large! Pattern-Case %3d, Line-Case: %2d;    Sidelengths: ab: %6.2f, bc: %6.2f, ca: %6.2f\n"
					"Vertices:\n"
					"CornerData:\n"
					"[0]: " FUCC ", " FUCCC "\n"
					"[1]: " FUCC ", " FUCCC "\n"
					"[2]: " FUCC ", " FUCCC "\n"
					"[3]: " FUCC ", " FUCCC "\n"
					"[4]: " FUCC ", " FUCCC "\n"
					"[5]: " FUCC ", " FUCCC "\n"
					"[6]: " FUCC ", " FUCCC "\n"
					"[7]: " FUCC ", " FUCCC "\n"
					, triIndex, lookupIndex, lineIndex, ab, bc, ca
					, PRINT_VEC2(cornerData.cornerValues[0])
					, PRINT_VEC(cornerData.cornerPositions[0])
					, PRINT_VEC2(cornerData.cornerValues[1])
					, PRINT_VEC(cornerData.cornerPositions[1])
					, PRINT_VEC2(cornerData.cornerValues[2])
					, PRINT_VEC(cornerData.cornerPositions[2])
					, PRINT_VEC2(cornerData.cornerValues[3])
					, PRINT_VEC(cornerData.cornerPositions[3])
					, PRINT_VEC2(cornerData.cornerValues[4])
					, PRINT_VEC(cornerData.cornerPositions[4])
					, PRINT_VEC2(cornerData.cornerValues[5])
					, PRINT_VEC(cornerData.cornerPositions[5])
					, PRINT_VEC2(cornerData.cornerValues[6])
					, PRINT_VEC(cornerData.cornerPositions[6])
					, PRINT_VEC2(cornerData.cornerValues[7])
					, PRINT_VEC(cornerData.cornerPositions[7])
);
		}

		triangles[triIndex] = tri;
	}

	for(int p = 0; p < t; ++p) {
		int triIndex = ti.thread_id*5+p;
		Tri tri = triangles[triIndex];
		tri.aVorticity = tri.aVorticity * t;
		tri.bVorticity = tri.bVorticity * t;
		tri.cVorticity = tri.cVorticity * t;
	}
}

// ######################################################################### //
// ### LinkedCell for Marching Cube Voxels ################################# //
// ######################################################################### //

__global__
void createLinkedCellStructureForMarchingCubes(ArrayView<ParticlePositionRadius> particlePositions
		, ArrayView<int> hashBuckets
		, ArrayView<int> hashOverflow
		, float gridSize
		, vec3 boundingBoxMin
		, vec3 boundingBoxMax
		, int fluidsBegin
		, int fluidsEnd) {
	Saiga::CUDA::ThreadInfo<> ti;
	int particleID = ti.thread_id + fluidsBegin;
	if(particleID >= fluidsEnd) return;

	ParticlePositionRadius particlePosition;
	Saiga::CUDA::vectorCopy(particlePositions.data()+particleID, &particlePosition);

	int hash = positionToVoxelIndex(particlePosition.position, boundingBoxMin, boundingBoxMax, gridSize);

	hashOverflow[ti.thread_id] = atomicExch(&hashBuckets[hash], ti.thread_id);
}


// sets the iso-surface value for each voxel center of the bounding box
// called once for each voxel of the bounding box
__global__ void fillMarchingCubesGrid(float h
		, float sigma
		, ArrayView<ParticlePositionRadius> positions
		, ArrayView<ParticleMomentumMass> masses
		, ArrayView<FluidVorticity> vorticities
		, ArrayView<int> gridBuckets
		, ArrayView<int> gridOverflow
		, vec3 fluidBoundsMin
		, vec3 fluidBoundsMax
		, float gridSize
		, float fluidsBegin
		, ArrayView<vec2> gridValues) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= gridValues.size()) return;

	vec3 voxelCenter = indexToVoxelCenter(ti.thread_id, fluidBoundsMin, fluidBoundsMax, gridSize);

	// evaluate the surface funcition at this voxel's center
	vec2 value = surfaceFunction(voxelCenter, h, sigma, positions, masses, vorticities, gridBuckets, gridOverflow, fluidBoundsMin, fluidBoundsMax, fluidsBegin, gridSize);

	gridValues[ti.thread_id] = value;
}

__global__ void fillMarchingCubesGridPerParticle(float h
		, float sigma
		, ArrayView<ParticlePositionRadius> positions
		, ArrayView<ParticleMomentumMass> masses
		, ArrayView<FluidVorticity> vorticities
		, ArrayView<int> gridBuckets
		, ArrayView<int> gridOverflow
		, vec3 fluidBoundsMin
		, vec3 fluidBoundsMax
		, float gridSize
		, float fluidsBegin
		, float fluidsEnd
		, int gauss
		, ArrayView<vec2> gridValues) {
	Saiga::CUDA::ThreadInfo<> ti;
	int particleID = ti.thread_id + fluidsBegin;
	if(particleID >= fluidsEnd) return;

	ParticlePositionRadius particle;
	Saiga::CUDA::vectorCopy(positions.data()+particleID, &particle);

	int voxelID = positionToVoxelIndex(particle.position, fluidBoundsMin, fluidBoundsMax, gridSize);
	ivec3 voxel3D = linearTo3DIndex(voxelID, fluidBoundsMin, fluidBoundsMax, gridSize);

	for(int x = -gauss; x <= gauss; ++x) {
		for(int y = -gauss; y <= gauss; ++y) {
			for(int z = -gauss; z <= gauss; ++z) {
				ivec3 offset(x,y,z);

				int linearVoxel = index3DToLinear(voxel3D+offset, fluidBoundsMin, fluidBoundsMax, gridSize);
				if(linearVoxel < 0 || linearVoxel >= gridValues.size()) continue;
				vec3 voxelCenter = indexToVoxelCenter(linearVoxel, fluidBoundsMin, fluidBoundsMax, gridSize);

				vec2 value = surfaceFunction(particle.position, h, sigma, positions, masses, vorticities, gridBuckets, gridOverflow, fluidBoundsMin, fluidBoundsMax, fluidsBegin, gridSize);

				float distance = (particle.position - voxelCenter).norm();
				float factor = distance > 0 ? 1.f / distance : 1.f;

				atomicAdd(&gridValues[linearVoxel].x(), value.x() * factor);
				atomicAdd(&gridValues[linearVoxel].y(), value.y() * factor);
			}
		}
	}

}

__global__ void normalizeVoxelValues(ArrayView<vec2> gridValues, ArrayView<int> gridBuckets, ArrayView<int> gridOverflow) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= gridValues.size()) return;

	int numParticles = 0;
	for(int neighborID = gridBuckets[ti.thread_id]; neighborID != -1; neighborID = gridOverflow[neighborID]) {
		++numParticles;
	}

	if(numParticles > 0) {
		gridValues[ti.thread_id] /= numParticles;
	}
}

__device__ inline bool vec3AreEqual(vec3 a, vec3 b) {
	vec3 diff = a - b;
	return diff.norm() < 0.001f;
}

__device__ inline vec3 computeFaceNormal(Tri tri) {
	vec3 a, b;
	a = tri.b - tri.a;
	b = tri.c - tri.a;
	return a.cross(b).normalized();
}

__device__ inline int signum(float value) {
	return value < 0.f ? -1 : 1;
}

__device__ vec3 mapHelperVoxelCenter(ivec3 index3D, vec3 bbMin, float gridSize) {
	// start at the origin of the bounding box
	vec3 minCorner = bbMin;
	// move to the min corner of this voxel by adding the 3d-index times the grid size
	minCorner += vec3((float)index3D.x(), (float)index3D.y(), (float)index3D.z()) * gridSize;

	// add half the gridSize in each ordinal direction to get the center
	vec3 voxelCenter = minCorner + vec3(gridSize, gridSize, gridSize) * 0.5f;

	return voxelCenter;
}

__device__ inline int mapVertexToEdge(vec3 vertex, int voxelIndex, vec3 bbMin, vec3 bbMax, float gridSize) {
	ivec3 index3D = linearTo3DIndexBigger(voxelIndex, bbMin, bbMax, gridSize, ivec3(1,1,1));

	vec3 voxelCenter = mapHelperVoxelCenter(index3D, bbMin, gridSize);

	// ################################################# //
	// ### compute the correct voxel for this vertex ### //
	vec3 diffCenter = vertex - voxelCenter;

	// compute the corresponding edge for this vertex
	ivec3 offset = ivec3(0,0,0);
	diffCenter *= 2.f/gridSize;

	offset.x() = (diffCenter.x() < 1.f - 0.001f) ? 0 : 1;
	offset.y() = (diffCenter.y() < 1.f - 0.001f) ? 0 : 1;
	offset.z() = (diffCenter.z() < 1.f - 0.001f) ? 0 : 1;

	ivec3 move = offset;

	// find out if this vertex is part of the x-, y- or z-component of the target voxel
	// ########################################################################### //
	// ### compute the dimension of the edge of the vertex in the target voxel ### //
	ivec3 targetCell = move + index3D;
	vec3 targetVoxelCenter = mapHelperVoxelCenter(targetCell, bbMin, gridSize);
	vec3 targetVoxelMin = targetVoxelCenter - vec3(gridSize, gridSize, gridSize)*0.5f;

	vec3 diffTargetCenter = targetVoxelMin - vertex;
	int idx;
	if(diffTargetCenter.x() < 0.001f) {
		idx = 0;
	} else if(diffTargetCenter.y() < 0.001f) {
		idx = 1;
	} else if(diffTargetCenter.z() < 0.001f) {
		idx = 2;
	} else {
		printf("Ahem... idx: %d, targetCellLinear: %d\n", idx, index3DToLinearBigger(targetCell, bbMin, bbMax, gridSize, ivec3(1,1,1)));
	}

	int edgeIndex = index3DToLinearBigger(targetCell, bbMin, bbMax, gridSize, ivec3(1,1,1));
	edgeIndex *= 3;
	edgeIndex += idx;

	return edgeIndex;
}

__global__ void createEdgeIndices(ArrayView<Tri> triangles, ArrayView<int> edgeIndices, ArrayView<int> validTriangles, vec3 bbMin, vec3 bbMax, float gridSize) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id*5 >= triangles.size()) return;

	for(int i = 0; i < 5; ++i) {
		int index = ti.thread_id*5+i;
		if(!validTriangles[index]) break;

		Tri tri = triangles[index];

		tri.aIndex = mapVertexToEdge(tri.a, ti.thread_id, bbMin, bbMax, gridSize);
		edgeIndices[tri.aIndex] = 0;
		tri.bIndex = mapVertexToEdge(tri.b, ti.thread_id, bbMin, bbMax, gridSize);
		edgeIndices[tri.bIndex] = 0;
		tri.cIndex = mapVertexToEdge(tri.c, ti.thread_id, bbMin, bbMax, gridSize);
		edgeIndices[tri.cIndex] = 0;

		triangles[index] = tri;
	}
}


__global__ void updateIndices(ArrayView<Tri> triangles, ArrayView<int> edgeIndexOffsets, ArrayView<int> validTriangles) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id*5 >= triangles.size()) return;

	for(int i = 0; i < 5; ++i) {
		int index = ti.thread_id*5+i;
		if(!validTriangles[index]) break;

		Tri tri = triangles[index];

		// set the triangle indices
		tri.aIndex += edgeIndexOffsets[tri.aIndex];
		tri.bIndex += edgeIndexOffsets[tri.bIndex];
		tri.cIndex += edgeIndexOffsets[tri.cIndex];

		triangles[index] = tri;
	}
}

__device__ inline void updateTriangleInner(Tri& tri, Tri neighbor, int& aNum, int& bNum, int& cNum) {
	if(tri.aIndex == neighbor.aIndex || tri.aIndex == neighbor.bIndex || tri.aIndex == neighbor.cIndex) {
		tri.aNormal += computeFaceNormal(neighbor);

		tri.aSmoothed += neighbor.a;
		tri.aSmoothed += neighbor.b;
		tri.aSmoothed += neighbor.c;
		tri.aVorticity += neighbor.aVorticity;
		tri.aVorticity += neighbor.bVorticity;
		tri.aVorticity += neighbor.cVorticity;
		aNum += 3;
	}

	if(tri.bIndex == neighbor.aIndex || tri.bIndex == neighbor.bIndex || tri.bIndex == neighbor.cIndex) {
		tri.bNormal += computeFaceNormal(neighbor);

		tri.bSmoothed += neighbor.a;
		tri.bSmoothed += neighbor.b;
		tri.bSmoothed += neighbor.c;
		tri.bVorticity += neighbor.aVorticity;
		tri.bVorticity += neighbor.bVorticity;
		tri.bVorticity += neighbor.cVorticity;
		bNum += 3;
	}

	if(tri.cIndex == neighbor.aIndex || tri.cIndex == neighbor.bIndex || tri.cIndex == neighbor.cIndex) {
		tri.cNormal += computeFaceNormal(neighbor);

		tri.cSmoothed += neighbor.a;
		tri.cSmoothed += neighbor.b;
		tri.cSmoothed += neighbor.c;
		tri.cVorticity += neighbor.aVorticity;
		tri.cVorticity += neighbor.bVorticity;
		tri.cVorticity += neighbor.cVorticity;
		cNum += 3;
	}
}

__device__ inline void updateTriangle(Tri& tri, int neighborIndex, ArrayView<Tri> triangles, ArrayView<int> validTriangles, int& aNum, int& bNum, int& cNum) {
	for(int i = 0; i < 5; ++i) {
		int triIndex = neighborIndex*5+i;
		if(!validTriangles[triIndex]) return;
		Tri neighbor = triangles[triIndex];
		updateTriangleInner(tri, neighbor, aNum, bNum, cNum);
	}
}

__global__ void computeVertexNormals(ArrayView<Tri> triangles, ArrayView<int> validTriangles, vec3 bbMin, vec3 bbMax, float gridSize) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id*5 >= triangles.size()) return;

	ivec3 index3D = linearTo3DIndex(ti.thread_id, bbMin, bbMax, gridSize);

	const ivec3 offsets[27] = {
		ivec3(-1,-1,-1),
		ivec3( 0,-1,-1),
		ivec3( 1,-1,-1),
		ivec3(-1, 0,-1),
		ivec3( 0, 0,-1),
		ivec3( 1, 0,-1),
		ivec3(-1, 1,-1),
		ivec3( 0, 1,-1),
		ivec3( 1, 1,-1),

		ivec3(-1,-1, 0),
		ivec3( 0,-1, 0),
		ivec3( 1,-1, 0),
		ivec3(-1, 0, 0),
		ivec3( 0, 0, 0),
		ivec3( 1, 0, 0),
		ivec3(-1, 1, 0),
		ivec3( 0, 1, 0),
		ivec3( 1, 1, 0),

		ivec3(-1,-1, 1),
		ivec3( 0,-1, 1),
		ivec3( 1,-1, 1),
		ivec3(-1, 0, 1),
		ivec3( 0, 0, 1),
		ivec3( 1, 0, 1),
		ivec3(-1, 1, 1),
		ivec3( 0, 1, 1),
		ivec3( 1, 1, 1)
	};

	// check neighboring cells for triangles that are adjacent to this voxel's triangles
	for(int i = 0; i < 5; ++i) {
		int index = ti.thread_id*5+i;
		if(!validTriangles[index]) break;

		Tri triangle = triangles[index];

		triangle.aNormal = vec3(0,0,0);
		triangle.aSmoothed = triangle.a;
		int aNum = 1;
		triangle.bNormal = vec3(0,0,0);
		triangle.bSmoothed = triangle.b;
		int bNum = 1;
		triangle.cNormal = vec3(0,0,0);
		triangle.cSmoothed = triangle.c;
		int cNum = 1;

		for(int k = 0; k < 27; ++k) {
			int index = index3DToLinear(index3D+offsets[k], bbMin, bbMax, gridSize);
			if(index > -1) {
				updateTriangle(triangle, index, triangles, validTriangles, aNum, bNum, cNum);
			}
		}
		triangle.aSmoothed /= (float)aNum;
		triangle.bSmoothed /= (float)bNum;
		triangle.cSmoothed /= (float)cNum;


		triangle.aNormal.normalize();
		triangle.bNormal.normalize();
		triangle.cNormal.normalize();

		triangle.aVorticity /= (float)aNum;
		triangle.bVorticity /= (float)aNum;
		triangle.cVorticity /= (float)aNum;

		// write the triangle back
		triangles[index] = triangle;
	}
}

__global__ void fillData(ArrayView<Tri> triangles, ArrayView<int> validTriangles, ArrayView<int> lel, ArrayView<WaterVertex> vertices, ArrayView<GLuint> indices) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= triangles.size()) return;

	if(!validTriangles[ti.thread_id]) return;

	// fetch triangle of this thread
	Tri tri = triangles[ti.thread_id];

	// write triangle indices
	auto toll = lel[ti.thread_id];
	if((toll)*3+2 >= indices.size()) {
		printf("Indices is too small: toll: %d, indices.size(): %lu\n", (toll)*3+2, indices.size());
	}
	indices[toll*3+0] = tri.aIndex;
	indices[toll*3+1] = tri.bIndex;
	indices[toll*3+2] = tri.cIndex;

	if(tri.aIndex >= vertices.size()) {
		printf("Index out of Bounds: tri.aIndex=%d, vertices.size()=%lu\n", tri.aIndex, vertices.size());
	}
	if(tri.bIndex >= vertices.size()) {
		printf("Index out of Bounds: tri.bIndex=%d, vertices.size()=%lu\n", tri.bIndex, vertices.size());
	}
	if(tri.cIndex >= vertices.size()) {
		printf("Index out of Bounds: tri.cIndex=%d, vertices.size()=%lu\n", tri.cIndex, vertices.size());
	}
	// write triangle vertices
	// FIXME: this is racy, but that shouldn't matter as every other thread writing at the same destination would write the same data
	vertices[tri.aIndex] = { vec4(PRINT_VEC(tri.a), 1), vec4(PRINT_VEC(tri.aNormal), 0), vec4(tri.aVorticity, 0.f, 1.f, 0.f) };
	vertices[tri.bIndex] = { vec4(PRINT_VEC(tri.b), 1), vec4(PRINT_VEC(tri.bNormal), 0), vec4(tri.aVorticity, 1.f, 0.f, 0.f) };
	vertices[tri.cIndex] = { vec4(PRINT_VEC(tri.c), 1), vec4(PRINT_VEC(tri.cNormal), 0), vec4(tri.aVorticity, 0.f, 0.f, 0.f) };
}

__global__ void smoothenVertices(ArrayView<Tri> triangles, ArrayView<int> validTriangles) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= triangles.size()) return;

	if(!validTriangles[ti.thread_id]) return;

	Tri tri = triangles[ti.thread_id];
	tri.a *= 3.f;
	tri.a += tri.aSmoothed;
	tri.a *= 0.25f;

	tri.b *= 3.f;
	tri.b += tri.bSmoothed;
	tri.b *= 0.25f;

	tri.c *= 3.f;
	tri.c += tri.cSmoothed;
	tri.c *= 0.25f;
	triangles[ti.thread_id] = tri;
}

// ######################################################################### //
// ### C++ Functionality ################################################### //
// ######################################################################### //

struct SAIGA_ALIGN(32) DoublePosition {
	ParticlePositionRadius min = { vec3(INFINITY, INFINITY, INFINITY), 1.f };
	ParticlePositionRadius max = { vec3(-INFINITY, -INFINITY, -INFINITY), 1.f };
};

struct ExtremeReducerUnary {
	__host__ __device__
		DoublePosition operator()(ParticlePositionRadius x) const {
			return {x, x};
		}
};

struct ExtremeReducer {
	__host__ __device__
		DoublePosition operator()(DoublePosition x, DoublePosition y) {
			DoublePosition result;
			result.min.position.x() = fmin(x.min.position.x(), y.min.position.x());
			result.min.position.y() = fmin(x.min.position.y(), y.min.position.y());
			result.min.position.z() = fmin(x.min.position.z(), y.min.position.z());

			result.max.position.x() = fmax(x.max.position.x(), y.max.position.x());
			result.max.position.y() = fmax(x.max.position.y(), y.max.position.y());
			result.max.position.z() = fmax(x.max.position.z(), y.max.position.z());
			return result;
		}
};

/**
 *	computes a bounding box of the fluid particles
 */
FluidBounds ParticleSystem::computeFluidBounds(float gridSize) {
	int fluidsBegin = particlesBegin(ParticleType::FluidParticle);
	int fluidsEnd = particlesEnd(ParticleType::FluidParticle);

	vec3 min, max;
	{
		PROFILE_SCOPE("Transform Reduce");

		DoublePosition minmax = thrust::transform_reduce(m_particles.d_positionRadius.device_begin()+fluidsBegin, m_particles.d_positionRadius.device_begin()+fluidsEnd, ExtremeReducerUnary(), DoublePosition(), ExtremeReducer());

		min = minmax.min.position;
		max = minmax.max.position;
	}

	// increase the size of the bounding box by 2 voxels in each dimension
	max += vec3(gridSize, gridSize, gridSize)*2.f;
	min -= vec3(gridSize, gridSize, gridSize)*2.f;

	FluidBounds bounds;
	bounds.min = min;
	bounds.max = max;

	return bounds;
}

// ######################################################################### //
// ### Mesh Creation ####################################################### //
// ######################################################################### //

inline void setupTessShader(std::shared_ptr<Saiga::MVPShader>& shader) {
	ImGuiOptions& options = ImGuiOptions::get();

	shader->bind();
	GLint location = shader->getUniformLocation("u_tessLevelOuter");
	shader->upload(location, options.tessLevelOuter);
	location = shader->getUniformLocation("u_tessLevelInner");
	shader->upload(location, options.tessLevelInner);
	location = shader->getUniformLocation("u_tessAlpha");
	shader->upload(location, options.tessAlpha);
	shader->unbind();
}

inline int addVertexToMesh(const vec3& vertex, Saiga::TriangleMesh<VertexNT, GLuint>& mesh, std::unordered_map<KeyVec3, int>& cache) {
	int index = -1;
	KeyVec3 key;
	key.key = vertex;
	auto position = cache.find(key);
	if(position == cache.end()) {
		index = mesh.addVertex(VertexNT(vertex));
		cache[key] = index;
	} else {
		index = position->second;
	}
	return index;
}

void ParticleSystem::resizeBuffers(ivec3 voxelDimensions, int numFluidParticles, ArrayView<vec2>& voxelValues, ArrayView<int>& buckets, ArrayView<int>& overflows, ArrayView<Tri>& triangles, ArrayView<int>& edgeIndices, ArrayView<int>& validTriangles) {
	PROFILE_FUNCTION();

	// allow a maximum of 12'000'000 voxels
	const int SIZE = MAX_VOXELS;
	// allow a maximum of 300'000 fluid particles
	const int PARTICLESIZE = MAX_FLUIDS;

	int numVoxels = voxelDimensions.x()*voxelDimensions.y()*voxelDimensions.z();

	{
		PROFILE_SCOPE("Voxel Values");

		d_marchingCubeVoxelValues.resize(SIZE);
		voxelValues = ArrayView<vec2>(d_marchingCubeVoxelValues.data().get(), numVoxels);
	}

	{
		PROFILE_SCOPE("Buckets");

		// resize marchingCubeBuckets to the number of voxels and initialize them with -1
		d_marchingCubeBuckets.resize(SIZE);
		thrust::fill(d_marchingCubeBuckets.begin(), d_marchingCubeBuckets.end(), -1);
		buckets = ArrayView<int>(d_marchingCubeBuckets.data().get(), numVoxels);
	}

	{
		PROFILE_SCOPE("Overflows");

		// resize marchingCubeOverflow to the number of fluid particles and fill it with [fluidsBegin, fluidsEnd)
		d_marchingCubeOverflow.resize(PARTICLESIZE);
		thrust::sequence(d_marchingCubeOverflow.begin(), d_marchingCubeOverflow.end(), 0);
		overflows = ArrayView<int>(d_marchingCubeOverflow.data().get(), numFluidParticles);
	}

	{
		PROFILE_SCOPE("Triangles");

		// resize marchingCubeTriangles to the maximum amount of possible triangles
		d_marchingCubeTriangles.resize(SIZE);
		triangles = ArrayView<Tri>(d_marchingCubeTriangles.data().get(), numVoxels * 5);
	}

	{
		PROFILE_SCOPE("Edge Indices");

		// resize marching edges
		int bigger = (voxelDimensions.x()+1)*(voxelDimensions.y()+1)*(voxelDimensions.z()+1)*3;
		d_marchingEdgeIndices.resize(SIZE);
		thrust::fill(d_marchingEdgeIndices.begin(), d_marchingEdgeIndices.end(), -1);
		edgeIndices = ArrayView<int>(d_marchingEdgeIndices.data().get(), bigger);
	}

	{
		PROFILE_SCOPE("Valid Triangles");

		// resize valid triangles
		d_validTriangles.resize(SIZE, 0);
		thrust::fill(d_validTriangles.begin(), d_validTriangles.end(), 0);
		validTriangles = ArrayView<int>(d_validTriangles.data().get(), numVoxels*5);
	}

	{
		PROFILE_SCOPE("LEL");

		lel.resize(SIZE);
	}
}

void ParticleSystem::doMarchingCubes() {
	int numFluidParticles = getNumParticles(ParticleType::FluidParticle);
	if(numFluidParticles < 1) {
		// do nothing if no fluid particles exist
		return;
	}

	ImGuiOptions& options = ImGuiOptions::get();
	if(!options.renderFluidMesh) {
		// no need to compute this mesh
		return;
	}

	// get the bounds of the fluids
	FluidBounds bounds;
	{
		PROFILE_SCOPE("Fluid Bounds");

		bounds = computeFluidBounds(options.marchingCubesGridStep);
	}

	// compute how many grid voxels will be generated:
	ivec3 voxelDimensions = computeNumVoxels(bounds.min, bounds.max, options.marchingCubesGridStep);
	long long numVoxels = voxelDimensions.x() * voxelDimensions.y() * voxelDimensions.z();

	if(numVoxels >= MAX_VOXELS) {
		std::cerr << "The graphics memory is not big enough for this large amounts of water!" << std::endl;
		return;
	}

	if(numVoxels < 1) {
		// there is nothing to do here, except for resetting the water mesh
		// TODO: reset the mesh
		return;
	}

	// get fluid particle iterators
	int fluidsBegin = particlesBegin(ParticleType::FluidParticle);
	int fluidsEnd = particlesEnd(ParticleType::FluidParticle);

	ArrayView<vec2> voxelValues;
	ArrayView<int> buckets;
	ArrayView<int> overflows;
	ArrayView<Tri> triangles;
	ArrayView<int> edgeIndices;
	ArrayView<int> validTriangles;

	int numTriangles = 0;
	int numVertices = 0;

	// resize cubeVoxelValues to the number of voxels and initialize them with 0
	resizeBuffers(voxelDimensions, numFluidParticles, voxelValues, buckets, overflows, triangles, edgeIndices, validTriangles);

	if(options.enabledMarchingCubesSteps >= 1)
	{
		PROFILE_SCOPE("Linked Cell for Marching Cubes");

		createLinkedCellStructureForMarchingCubes<<<THREAD_BLOCK(numFluidParticles, BLOCK_SIZE)>>>(m_particles.d_positionRadius, buckets, overflows, options.marchingCubesGridStep, bounds.min, bounds.max, fluidsBegin, fluidsEnd);
		CUDA_SYNC_CHECK_ERROR();
	}

	if(options.enabledMarchingCubesSteps >= 2)
	{
		PROFILE_SCOPE("Fill Voxels");

		initField<<<THREAD_BLOCK(numVoxels, BLOCK_SIZE)>>>(voxelValues);
		CUDA_SYNC_CHECK_ERROR();

		// fill the marching-cubes-grid with the values of the implicit function that describes the fluid surface
		fillMarchingCubesGridPerParticle<<<THREAD_BLOCK(numFluidParticles, BLOCK_SIZE)>>>(options.marchingCubesH, options.marchingCubesSigma, m_particles.d_positionRadius, m_particles.d_momentumMass, m_fluids.d_voriticities, buckets, overflows, bounds.min, bounds.max, options.marchingCubesGridStep, fluidsBegin, fluidsEnd, options.gauss, voxelValues);
		CUDA_SYNC_CHECK_ERROR();

		normalizeVoxelValues<<<THREAD_BLOCK(numVoxels, BLOCK_SIZE)>>>(voxelValues, buckets, overflows);
		CUDA_SYNC_CHECK_ERROR();
	}

	if(options.enabledMarchingCubesSteps >= 3)
	{
		PROFILE_SCOPE("The actual Marching Cubes");

		// perform the triangulation by actually doing the marching cubes algorithm
		marchingCubesKernel<<<THREAD_BLOCK(numVoxels, BLOCK_SIZE)>>>(voxelValues, triangles, validTriangles, options.marchingCubesGridStep, bounds.min, bounds.max, options.marchingCubesIsoDensity);
		CUDA_SYNC_CHECK_ERROR();
	}

	if(options.enabledMarchingCubesSteps >= 4)
	{
		PROFILE_SCOPE("Edge Index Creation");

		// compute vertex indices
		createEdgeIndices<<<THREAD_BLOCK(numVoxels, BLOCK_SIZE)>>>(triangles, edgeIndices, validTriangles, bounds.min, bounds.max, options.marchingCubesGridStep);
		CUDA_SYNC_CHECK_ERROR();
	}

	if(options.enabledMarchingCubesSteps >= 5)
	{
		PROFILE_SCOPE("Inclusive Scan");

		if(edgeIndices.size() > 0) {
			thrust::inclusive_scan(edgeIndices.device_begin(), edgeIndices.device_end(), edgeIndices.device_begin());
			CUDA_SYNC_CHECK_ERROR();

			int lastEdgeIndex;
			thrust::copy(edgeIndices.device_end()-1, edgeIndices.device_end(), &lastEdgeIndex);
			CUDA_SYNC_CHECK_ERROR();
			numVertices = edgeIndices.size() + lastEdgeIndex;
		} else {
			numVertices = 0;
		}
	}

	if(options.enabledMarchingCubesSteps >= 6)
	{
		PROFILE_SCOPE("Index Computation");
		updateIndices<<<THREAD_BLOCK(numVoxels, BLOCK_SIZE)>>>(triangles, edgeIndices, validTriangles);
		CUDA_SYNC_CHECK_ERROR();
	}

	if(options.enabledMarchingCubesSteps >= 7)
	{
		PROFILE_SCOPE("Compute Vertex Normals");

		// compute vertex normals
		computeVertexNormals<<<THREAD_BLOCK(numVoxels, BLOCK_SIZE)>>>(triangles, validTriangles, bounds.min, bounds.max, options.marchingCubesGridStep);
		CUDA_SYNC_CHECK_ERROR();
	}

	if(options.enabledMarchingCubesSteps >= 8)
	{
		PROFILE_SCOPE("Vertex Smoothing");
		smoothenVertices<<<THREAD_BLOCK(triangles.size(), BLOCK_SIZE)>>>(triangles, validTriangles);
		CUDA_SYNC_CHECK_ERROR();
	}

	ArrayView<int> lelTmp;
	if(options.enabledMarchingCubesSteps >= 9)
	{
		PROFILE_SCOPE("Computing Number of Triangles");

		lelTmp = ArrayView<int>(lel.data().get(), validTriangles.size());
		thrust::inclusive_scan(validTriangles.device_begin(), validTriangles.device_end(), lelTmp.device_begin(), thrust::plus<int>());
		CUDA_SYNC_CHECK_ERROR();
		thrust::copy(lelTmp.device_end()-1, lelTmp.device_end(), &numTriangles);
		CUDA_SYNC_CHECK_ERROR();
		numTriangles++;
	}

	if(options.enabledMarchingCubesSteps >= 10)
	{
		PROFILE_SCOPE("Moving Data to GL Buffers");

		WaterRenderer::getInstance().getMesh().getBuffer().resize(numVertices, numTriangles*3, GL_DYNAMIC_DRAW);

		if(WaterRenderer::getInstance().getMesh().getBuffer().isEverythingAwesome()) {
			auto vertices = WaterRenderer::getInstance().getMesh().getBuffer().getMappedVertexBuffer();
			auto indices = WaterRenderer::getInstance().getMesh().getBuffer().getMappedIndexBuffer();

			fillData<<<THREAD_BLOCK(triangles.size(), BLOCK_SIZE)>>>(triangles, validTriangles, lelTmp, vertices, indices);
			CUDA_SYNC_CHECK_ERROR();

			WaterRenderer::getInstance().getMesh().getBuffer().unmapVertexBuffer();
			WaterRenderer::getInstance().getMesh().getBuffer().unmapIndexBuffer();
		}
	}
}
