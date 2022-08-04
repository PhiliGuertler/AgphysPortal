#include "particleSystem_fluids.h"

#include "../predicates.h"

#include "saiga/cuda/thread_info.h"
#include "saiga/cuda/memory.h"

class ConstVec3 {
	public:
		__device__ __host__ ConstVec3() {}
		__device__ __host__ ~ConstVec3() {}

		float x,y,z;
		__device__ __host__ operator vec3() const { return vec3(x,y,z); }
};

__constant__ ConstVec3 constNeighborOffsets[27];

__constant__ float distr[] =
{
	-0.34828757091811f, -0.64246175794046f, -0.15712936555833f, -0.28922267225069f, 0.70090742209037f,
	0.54293139350737f, 0.86755128105523f, 0.68346917800767f, -0.74589352018474f, 0.39762042062246f,
	-0.70243115988673f, -0.85088539675385f, -0.25780126697281f, 0.61167922970451f, -0.8751634423971f,
	-0.12334015086449f, 0.10898816916579f, -0.97167591190509f, 0.89839695948101f, -0.71134930649369f,
	-0.33928178406287f, -0.27579196788175f, -0.5057460942798f, 0.2341509513716f, 0.97802030852904f,
	0.49743173248015f, -0.92212845381448f, 0.088328595779989f, -0.70214782175708f, -0.67050553191011f
};



// ######################################################################### //
// ### CUDA Kernels ######################################################## //
// ######################################################################### //

static __device__ inline float quickNorm(vec3 vec) {
	return sqrt(vec.dot(vec));
}

static __device__ inline vec3 quickNormalize(vec3 vec) {
	return vec.normalized();
}

// only works for exponents greater than 0
static __device__ inline float quickPow(float base, int exponent) {
	float result = 0.f;
	for(int i = 0; i < exponent; ++i) {
		result *= base;
	}
	return result;
}

static __device__ inline float kernelPoly6(vec3 r, float h, float hToTheNegativeNinth) {
	float factor = (315.f / (64.f * M_PI)) * hToTheNegativeNinth;
	float result = 0.f;
	if(r.dot(r) <= h*h) {
		float rip = h*h - r.dot(r);
		result = factor * rip * rip * rip;
	}
	return result;
}

static __device__ inline vec3 kernelSpiky(vec3 r, float h, float hToTheNegativeSixth) {
	float factor = (-45.f / M_PI) *hToTheNegativeSixth;
	vec3 result = vec3(0.f,0.f,0.f);
	float norm = quickNorm(r);
	if(norm <= h && norm > 1e-4) {
		float rip = h - norm;
		result = factor * (r/norm) * rip*rip;
	}
	return result;
}

__global__ void guessDensity(ArrayView<ParticlePositionRadius> guessedPositions
		, ArrayView<ParticleMomentumMass> moms
		, ArrayView<int> numParticlesPerBucket
		, ArrayView<int> firstParticleOfBucket
		, ArrayView<FluidLambda> lambdas
		, ArrayView<FluidVorticity> vorticities
		, int fluidsBegin
		, int fluidsEnd
		, float h
		, float k
		, float q
		, float epsilon
		, int n
		, float ro0) {

	Saiga::CUDA::ThreadInfo<> ti;
	int fluidParticleID = fluidsBegin + ti.thread_id;
	if(fluidParticleID >= fluidsEnd) return;

	// get current particle's data
	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+fluidParticleID, &guessedPos);

	FluidLambda lambda;
	Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+fluidParticleID, &lambda);

	// iterate over neighbors
	float ro = 0.f;

	float spikyThingInner = 0.f;
	vec3 spikyThingOuter = vec3(0,0,0);

	// FIXME: these can be moved to the caller and does not need to be computed for each particle
	float hhh = h*h*h;
	float hToTheSixth = hhh*hhh;
	float hToTheNinth = hToTheSixth*hhh;
	hToTheSixth = 1.f / hToTheSixth;
	hToTheNinth = 1.f / hToTheNinth;

	for(int i = 0; i < 27; ++i) {
		vec3 offset = constNeighborOffsets[i];
		int neighborHash = hashMe(guessedPos.position + offset*h, h, firstParticleOfBucket.size());
		int firstIndex = firstParticleOfBucket[neighborHash];
		int numParticles = numParticlesPerBucket[neighborHash];
		for(int j = 0; j < numParticles; ++j) {
			int neighborID = j + firstIndex;
			FluidLambda fluidLambdaOther;
			Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+neighborID, &fluidLambdaOther);

			if(fluidLambdaOther.isFluid == 0) {
				// only consider fluid particles
				//continue;
			}

			// load neighbor particle
			ParticlePositionRadius neighborPos;
			Saiga::CUDA::vectorCopy(guessedPositions.data()+neighborID, &neighborPos);

			vec3 difference = guessedPos.position - neighborPos.position;

			if(difference.dot(difference) >= h*h) {
				continue;
			}

			ParticleMomentumMass neighborMom;
			Saiga::CUDA::vectorCopy(moms.data()+neighborID, &neighborMom);


			ro += neighborMom.massinv * kernelPoly6(difference, h, hToTheNinth); 

			vec3 yikes = kernelSpiky(difference, h, hToTheSixth);
			yikes /= ro0;
			spikyThingOuter += yikes;
			spikyThingInner += yikes.dot(yikes);

		}
	}

	float nablaC = spikyThingOuter.dot(spikyThingOuter) + spikyThingInner;
	float c_i = (ro / ro0) - 1.f;

	lambda.lambda = -c_i / (nablaC + epsilon);

	Saiga::CUDA::vectorCopy<FluidLambda, int2>(&lambda, lambdas.data()+fluidParticleID);

	FluidVorticity vort;
	Saiga::CUDA::vectorCopy(vorticities.data()+fluidParticleID, &vort);
	vort.density = ro;
	Saiga::CUDA::vectorCopy(&vort, vorticities.data()+fluidParticleID);
}

__global__ void fluidPositionUpdate(ArrayView<ParticleDeltaValues> deltas
		, ArrayView<ParticlePositionRadius> guessedPositions
		, ArrayView<FluidLambda> lambdas
		, ArrayView<int> numParticlesPerBucket
		, ArrayView<int> firstParticleOfBucket
		, int fluidsBegin
		, int fluidsEnd
		, float h
		, float k
		, float q
		, int n
		, float ro0) {
	Saiga::CUDA::ThreadInfo<> ti;
	int fluidParticleID = fluidsBegin + ti.thread_id;
	if(fluidParticleID >= fluidsEnd) return;

	ParticleDeltaValues delta;
	Saiga::CUDA::vectorCopy(deltas.data()+fluidParticleID, &delta);

	FluidLambda lambda;
	Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+fluidParticleID, &lambda);

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+fluidParticleID, &guessedPos);

	vec3 sum = vec3(0.f,0.f,0.f);

	float hhh = h*h*h;
	float hToTheSixth = hhh*hhh;
	float hToTheNinth = hToTheSixth*hhh;
	hToTheSixth = 1.f / hToTheSixth;
	hToTheNinth = 1.f / hToTheNinth;

	vec3 deltaQ = vec3(0,q,0);
	for(int i = 0; i < 27; ++i) {
		vec3 offset = constNeighborOffsets[i];
		int neighborHash = hashMe(guessedPos.position + offset*h, h, firstParticleOfBucket.size());
		int firstIndex = firstParticleOfBucket[neighborHash];
		int numParticles = numParticlesPerBucket[neighborHash];
		for(int j = 0; j < numParticles; ++j) {
			int neighborID = j + firstIndex;
			FluidLambda neighborLambda;
			Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+neighborID, &neighborLambda);
			
			if(neighborLambda.isFluid == 0) {
				// only consider fluid particles
				continue;
			}

			ParticlePositionRadius neighborGuessedPos;
			Saiga::CUDA::vectorCopy(guessedPositions.data()+neighborID, &neighborGuessedPos);

			vec3 difference = guessedPos.position - neighborGuessedPos.position;

			if(difference.dot(difference) >= h*h) {
				continue;
			}

			float poly = kernelPoly6(difference, h, hToTheNinth) / kernelPoly6(deltaQ, h, hToTheNinth);
			float scorr = -k * quickPow(poly, n);

			sum += (lambda.lambda + neighborLambda.lambda + scorr) * kernelSpiky(guessedPos.position - neighborGuessedPos.position, h, hToTheSixth);
		}
	}

	sum /= ro0;

	delta.delta += sum;
	Saiga::CUDA::vectorCopy(&delta, deltas.data()+fluidParticleID);
}

__global__ void computeVoritcityStep1(ArrayView<ParticleMomentumMass> moms
		, ArrayView<ParticlePositionRadius> guessedPositions
		, ArrayView<FluidLambda> lambdas
		, ArrayView<FluidVorticity> vorticities
		, ArrayView<FluidViscosity> viscosities
		, ArrayView<int> numParticlesPerBucket
		, ArrayView<int> firstParticleOfBucket
		, int fluidsBegin
		, int fluidsEnd
		, float h
		, float c) {
	Saiga::CUDA::ThreadInfo<> ti;
	int fluidParticleID = (int)fluidsBegin + ti.thread_id;
	if(fluidParticleID >= fluidsEnd) return;

	vec3 omega_i = vec3(0,0,0);

	// load velocity and position of this particle
	ParticleMomentumMass mom;
	Saiga::CUDA::vectorCopy(moms.data()+fluidParticleID, &mom);

	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+fluidParticleID, &guessedPos);

	vec3 viscosity = vec3(0.f,0.f,0.f);


	float hhh = h*h*h;
	float hToTheSixth = hhh*hhh;
	float hToTheNinth = hToTheSixth*hhh;
	hToTheSixth = 1.f / hToTheSixth;
	hToTheNinth = 1.f / hToTheNinth;

	// --- Compute Vorticity for this particle --- //
	for(int i = 0; i < 27; ++i) {
		vec3 offset = constNeighborOffsets[i];
		int neighborHash = hashMe(guessedPos.position + offset*h, h, firstParticleOfBucket.size());
		int firstIndex = firstParticleOfBucket[neighborHash];
		int numParticles = numParticlesPerBucket[neighborHash];
		for(int j = 0; j < numParticles; ++j) {
			int neighborID = j + firstIndex;
			FluidLambda neighborLambda;
			Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+neighborID, &neighborLambda);
			
			if(neighborLambda.isFluid == 0) {
				// only consider fluid particles
				continue;
			}


			// load velocity and position of the neighbor particle
			ParticleMomentumMass neighborMom;
			Saiga::CUDA::vectorCopy(moms.data()+neighborID, &neighborMom);

			ParticlePositionRadius guessedPosNeighbor;
			Saiga::CUDA::vectorCopy(guessedPositions.data()+neighborID, &guessedPosNeighbor);

			vec3 firstArg = neighborMom.momentumVelocity - mom.momentumVelocity;
			vec3 difference = guessedPos.position - guessedPosNeighbor.position;
			vec3 secondArg = kernelSpiky(difference, h, hToTheSixth);

			omega_i += firstArg.cross(secondArg);

			if(difference.dot(difference) >= h*h) {
				continue;
			}

			float poly = kernelPoly6(difference, h, hToTheNinth);
			viscosity += firstArg * poly;
		}
	}

	// store the result in the vorticities buffer
	FluidVorticity result;
	Saiga::CUDA::vectorCopy(vorticities.data()+fluidParticleID, &result);
	result.voriticity = omega_i;
	Saiga::CUDA::vectorCopy(&result, vorticities.data()+fluidParticleID);

	FluidViscosity viserys;
	viserys.viscosity = c * viscosity;
	Saiga::CUDA::vectorCopy(&viserys, viscosities.data()+fluidParticleID);
}

__global__ void computeVoritcityStep2(ArrayView<ParticlePositionRadius> guessedPositions
		, ArrayView<FluidLambda> lambdas
		, ArrayView<FluidVorticity> vorticities
		, ArrayView<ParticleMomentumMass> moms
		, ArrayView<int> numParticlesPerBucket
		, ArrayView<int> firstParticleOfBucket
		, float dt
		, int fluidsBegin
		, int fluidsEnd
		, float epsilon
		, float h) {
	Saiga::CUDA::ThreadInfo<> ti;
	int fluidParticleID = (int)fluidsBegin + ti.thread_id;
	if(fluidParticleID >= fluidsEnd) return;

	// load guessed position and vorticity of this particle
	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+fluidParticleID, &guessedPos);

	FluidVorticity voriticity;
	Saiga::CUDA::vectorCopy(vorticities.data()+fluidParticleID, &voriticity);

	ParticleMomentumMass mom;
	Saiga::CUDA::vectorCopy(moms.data()+fluidParticleID, &mom);

	vec3 nablaOmega_i = vec3(0.f,0.f,0.f);

	vec3 viscosity = vec3(0.f,0.f,0.f);

	int ownHash = hashMe(guessedPos.position, h, firstParticleOfBucket.size());

	float hhh = h*h*h;
	float hToTheSixth = hhh*hhh;
	hToTheSixth = 1.f / hToTheSixth;

	// --- Compute Vorticity for this particle --- //
	for(int i = 0; i < 27; ++i) {
		vec3 offset = constNeighborOffsets[i];
		int neighborHash = hashMe(guessedPos.position + offset*h, h, firstParticleOfBucket.size());
		if(i > 0 && neighborHash == ownHash) continue;
		int firstIndex = firstParticleOfBucket[neighborHash];
		int numParticles = numParticlesPerBucket[neighborHash];
		for(int j = 0; j < numParticles; ++j) {
			int neighborID = j + firstIndex;
			FluidLambda neighborLambda;
			Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+neighborID, &neighborLambda);

			if(neighborLambda.isFluid == 0) {
				// only consider fluid particles
				continue;
			}

			// load guessed position and vorticity of this particle
			ParticleMomentumMass neighborMom;
			Saiga::CUDA::vectorCopy(moms.data()+neighborID, &neighborMom);

			ParticlePositionRadius neighborGuessedPos;
			Saiga::CUDA::vectorCopy(guessedPositions.data()+neighborID, &neighborGuessedPos);

			FluidVorticity neighborVoriticity;
			Saiga::CUDA::vectorCopy(vorticities.data()+neighborID, &neighborVoriticity);

			float norm = quickNorm(neighborVoriticity.voriticity);
			vec3 difference = guessedPos.position - neighborGuessedPos.position;
			nablaOmega_i += norm * kernelSpiky(difference, h, hToTheSixth);
		}
	}

	// compute the force
	vec3 force = (epsilon * quickNormalize(nablaOmega_i)).cross(voriticity.voriticity);

	// update particle velocity (Vorticity)
	mom.momentumVelocity += force*mom.massinv*dt;
	Saiga::CUDA::vectorCopy(&mom, moms.data()+fluidParticleID);
}

__global__ void updateVelocities(ArrayView<ParticleMomentumMass> moms, ArrayView<FluidViscosity> viscosities) {
	Saiga::CUDA::ThreadInfo<> ti;
	if(ti.thread_id >= moms.size()) return;

	// load momentum and viscosity
	ParticleMomentumMass mom;
	Saiga::CUDA::vectorCopy(moms.data()+ti.thread_id, &mom);

	FluidViscosity viserys;
	Saiga::CUDA::vectorCopy(viscosities.data()+ti.thread_id, &viserys);

	// update velocity
	mom.momentumVelocity += viserys.viscosity;
	viserys.viscosity = vec3(0.f,0.f,0.f);

	// write back
	Saiga::CUDA::vectorCopy(&viserys, viscosities.data()+ti.thread_id);
	Saiga::CUDA::vectorCopy(&mom, moms.data()+ti.thread_id);
}

// ######################################################################### //
// ### C++ ParticleSystem ################################################## //
// ######################################################################### //

void ParticleSystem::initConstantMemory() {
	ConstVec3 neighborOffsets[27];
	neighborOffsets[0].x =  0.f; neighborOffsets[0].y =  0.f; neighborOffsets[0].z = 0.f;
	neighborOffsets[1].x =  1.f; neighborOffsets[1].y =  0.f; neighborOffsets[1].z = 0.f;
	neighborOffsets[2].x = -1.f; neighborOffsets[2].y =  0.f; neighborOffsets[2].z = 0.f;
	neighborOffsets[3].x =  0.f; neighborOffsets[3].y =  1.f; neighborOffsets[3].z = 0.f;
	neighborOffsets[4].x =  1.f; neighborOffsets[4].y =  1.f; neighborOffsets[4].z = 0.f;
	neighborOffsets[5].x = -1.f; neighborOffsets[5].y =  1.f; neighborOffsets[5].z = 0.f;
	neighborOffsets[6].x =  0.f; neighborOffsets[6].y = -1.f; neighborOffsets[6].z = 0.f;
	neighborOffsets[7].x =  1.f; neighborOffsets[7].y = -1.f; neighborOffsets[7].z = 0.f;
	neighborOffsets[8].x = -1.f; neighborOffsets[8].y = -1.f; neighborOffsets[8].z = 0.f;

	neighborOffsets[9].x  =  0.f; neighborOffsets[9].y  =  0.f; neighborOffsets[9].z  = 1.f;
	neighborOffsets[10].x =  1.f; neighborOffsets[10].y =  0.f; neighborOffsets[10].z = 1.f;
	neighborOffsets[11].x = -1.f; neighborOffsets[11].y =  0.f; neighborOffsets[11].z = 1.f;
	neighborOffsets[12].x =  0.f; neighborOffsets[12].y =  1.f; neighborOffsets[12].z = 1.f;
	neighborOffsets[13].x =  1.f; neighborOffsets[13].y =  1.f; neighborOffsets[13].z = 1.f;
	neighborOffsets[14].x = -1.f; neighborOffsets[14].y =  1.f; neighborOffsets[14].z = 1.f;
	neighborOffsets[15].x =  0.f; neighborOffsets[15].y = -1.f; neighborOffsets[15].z = 1.f;
	neighborOffsets[16].x =  1.f; neighborOffsets[16].y = -1.f; neighborOffsets[16].z = 1.f;
	neighborOffsets[17].x = -1.f; neighborOffsets[17].y = -1.f; neighborOffsets[17].z = 1.f;

	neighborOffsets[18].x =  0.f; neighborOffsets[18].y =  0.f; neighborOffsets[18].z = -1.f;
	neighborOffsets[19].x =  1.f; neighborOffsets[19].y =  0.f; neighborOffsets[19].z = -1.f;
	neighborOffsets[20].x = -1.f; neighborOffsets[20].y =  0.f; neighborOffsets[20].z = -1.f;
	neighborOffsets[21].x =  0.f; neighborOffsets[21].y =  1.f; neighborOffsets[21].z = -1.f;
	neighborOffsets[22].x =  1.f; neighborOffsets[22].y =  1.f; neighborOffsets[22].z = -1.f;
	neighborOffsets[23].x = -1.f; neighborOffsets[23].y =  1.f; neighborOffsets[23].z = -1.f;
	neighborOffsets[24].x =  0.f; neighborOffsets[24].y = -1.f; neighborOffsets[24].z = -1.f;
	neighborOffsets[25].x =  1.f; neighborOffsets[25].y = -1.f; neighborOffsets[25].z = -1.f;
	neighborOffsets[26].x = -1.f; neighborOffsets[26].y = -1.f; neighborOffsets[26].z = -1.f;

	cudaMemcpyToSymbol(constNeighborOffsets, neighborOffsets, sizeof(neighborOffsets));
}

void ParticleSystem::fluidStep() {
	if(getNumParticles(ParticleType::FluidParticle) > 0) {
		ImGuiOptions& options = ImGuiOptions::get();
		guessDensity<<<THREAD_BLOCK(getNumParticles(ParticleType::FluidParticle), BLOCK_SIZE)>>>(m_particles.d_guessedPosition
				, m_particles.d_momentumMass
				, d_numParticlesPerBucket
				, d_beginIndicesFluids
				, m_fluids.d_lambdas
				, m_fluids.d_voriticities
				, particlesBegin(ParticleType::FluidParticle)
				, particlesEnd(ParticleType::FluidParticle)
				, options.fluidH
				, options.fluidK
				, options.fluidQ
				, options.fluidEpsilon
				, options.fluidN
				, options.fluidRo0);

		cudaDeviceSynchronize();

		fluidPositionUpdate<<<THREAD_BLOCK(getNumParticles(ParticleType::FluidParticle), BLOCK_SIZE)>>>(m_particles.d_deltaValues
				, m_particles.d_guessedPosition
				, m_fluids.d_lambdas
				, d_numParticlesPerBucket
				, d_beginIndicesFluids
				, particlesBegin(ParticleType::FluidParticle)
				, particlesEnd(ParticleType::FluidParticle)
				, options.fluidH
				, options.fluidK
				, options.fluidQ
				, options.fluidN
				, options.fluidRo0);

		cudaDeviceSynchronize();
	}
}

void ParticleSystem::vorticityStep(float dt) {
	ImGuiOptions& options = ImGuiOptions::get();
	if(getNumParticles(ParticleType::FluidParticle) > 0 && (options.enableViscosity || options.enableVorticity)) {
		computeVoritcityStep1<<<THREAD_BLOCK(getNumParticles(ParticleType::FluidParticle), BLOCK_SIZE)>>>(m_particles.d_momentumMass
				, m_particles.d_guessedPosition
				, m_fluids.d_lambdas
				, m_fluids.d_voriticities
				, m_fluids.d_viscosities
				, d_numParticlesPerBucket
				, d_beginIndicesFluids
				, particlesBegin(ParticleType::FluidParticle)
				, particlesEnd(ParticleType::FluidParticle)
				, options.fluidH
				, options.fluidC);
		cudaDeviceSynchronize();

		if(options.enableViscosity) {
			updateVelocities<<<THREAD_BLOCK(m_particleCounts.sum(), BLOCK_SIZE)>>>(m_particles.d_momentumMass, m_fluids.d_viscosities);
			cudaDeviceSynchronize();
		}

		if(!options.enableVorticity) {
			computeVoritcityStep2<<<THREAD_BLOCK(getNumParticles(ParticleType::FluidParticle), BLOCK_SIZE)>>>(m_particles.d_guessedPosition
					, m_fluids.d_lambdas
					, m_fluids.d_voriticities
					, m_particles.d_momentumMass
					, d_numParticlesPerBucket
					, d_beginIndicesFluids
					, dt
					, particlesBegin(ParticleType::FluidParticle)
					, particlesEnd(ParticleType::FluidParticle)
					, options.fluidVorticityEpsilon
					, options.fluidH);
		}
		cudaDeviceSynchronize();
	}
}


// ######################################################################### //
// ### Water Rendering Kernels ############################################# //
// ######################################################################### //

static __device__ float airPotential(vec3 guessedPosition1, vec3 guessedPosition2, float airRadius) {
	vec3 r = guessedPosition1 - guessedPosition2;
	float rLength = r.norm();
	if(rLength > airRadius || rLength < 0.0001f) {
		return 0.f;
	} else {
		return 1.f - (rLength / airRadius);
	}
}

// Mostly stolen from an implementation found online. This will probably not work on its own
__global__ void generateFoam(ArrayView<FluidLambda> lambdas
							, ArrayView<ParticlePositionRadius> guessedPositions
							, ArrayView<ParticleMomentumMass> velos
							, ArrayView<int> numParticlesPerBucket
							, ArrayView<int> firstParticleOfBucket
							// TODO: put these two into the regular particle device_vectors
							, ArrayView<ParticlePositionRadius> foamPositions
							, ArrayView<ParticleMomentumMass> foamVelocities
							, ArrayView<int> foamPhases	// ???
							, int fluidsBegin
							, int fluidsEnd
							, ArrayView<int> foamCount	// a device_vector with 1 entry
							, float restDistance
							, float airFactor
							, int numDiffuse	// ???
							, float h
							, float ro0) {
	Saiga::CUDA::ThreadInfo<> ti;
	int particleID = ti.thread_id + fluidsBegin;
	if(ti.thread_id >= fluidsEnd) return;

	if(foamPhases[particleID] != 0) return;
	if(foamCount[0] >= numDiffuse) return;
			

	// load position
	ParticlePositionRadius guessedPos;
	Saiga::CUDA::vectorCopy(guessedPositions.data()+particleID, &guessedPos);

	// load velocity
	ParticleMomentumMass velo;
	Saiga::CUDA::vectorCopy(velos.data()+particleID, &velo);

	// load lambda (which is used as density here)
	FluidLambda lambda;
	Saiga::CUDA::vectorCopy<FluidLambda, int2>(lambdas.data()+particleID, &lambda);
		
	int ownHash = hashMe(guessedPos.position, h, firstParticleOfBucket.size());
	
	// compute the velocity difference in the neighborhood
	float velocityDiff = 0.f;
	for(int i = 0; i < 27; ++i) {
		vec3 offset = constNeighborOffsets[i];
		int neighborHash = hashMe(guessedPos.position + offset*h, h, firstParticleOfBucket.size());

		// ignore the cell of this particle and all other cells that get mapped onto it
		if(neighborHash == ownHash) continue;

		int firstIndex = firstParticleOfBucket[neighborHash];
		int numParticles = numParticlesPerBucket[neighborHash];
		for(int j = 0; j < numParticles; ++j) {
			int neighborID = j + firstIndex;

			// load neighbor position
			ParticlePositionRadius guessedPosNeighbor;
			Saiga::CUDA::vectorCopy(guessedPositions.data()+neighborID, &guessedPosNeighbor);

			// load neighbor velocity
			ParticleMomentumMass veloNeighbor;
			Saiga::CUDA::vectorCopy(velos.data()+neighborID, &veloNeighbor);

			float airFact = airPotential(guessedPos.position, guessedPosNeighbor.position, airFactor);

			vec3 difference = guessedPos.position - guessedPosNeighbor.position;
			difference.normalize();

			vec3 veloVector = velo.momentumVelocity - veloNeighbor.momentumVelocity;

			velocityDiff += veloVector.norm() * (1.f - veloVector.normalized().dot(difference)) * airFact;
		}
	}

	float ek = 0.5f * quickPow(velo.momentumVelocity.norm(), 2);
	// ro0 is the resting density (1 in our case)
	float potential = velocityDiff * ek * max(1.f - (lambda.lambda / ro0), 0.f);

	// compute how many foam particles will be spawned
	int nd = 0;
	if(potential > 0.5f) {
		nd = min(20, numDiffuse - 1 - foamCount[0]);
	} else {
		return;
	}

	int count = atomicAdd(&foamCount[0], nd);
	count = min(count, numDiffuse - 1);
	int cap = min(count + nd, numDiffuse - 1);

	for(int i = count; i < cap; ++i) {
		float randX = distr[i % 30] * restDistance;
		float randY = distr[(i+1) % 30] * restDistance;
		float randZ = distr[(i+2) % 30] * restDistance;
		int rd = distr[particleID % 30] > 0.5f ? 1 : -1;

		vec3 rand = vec3(randX, randY, randZ) * rd;
		rand = guessedPos.position - rand;

		// write position of the foam particle
		ParticlePositionRadius foamPosition;
		foamPosition.position = rand;
		foamPosition.radius = 0.5f;
		Saiga::CUDA::vectorCopy(&foamPosition, foamPositions.data()+i);

		// write velocity of the foam particle, which is equal to this particle's velocity
		Saiga::CUDA::vectorCopy(&velo, foamVelocities.data()+i);
	}

	if(foamCount[0] >= numDiffuse) {
		atomicExch(&foamCount[0], numDiffuse - 1);
	}
}

// ######################################################################### //
// ### Reconstructing Surfaces of Particle Based Fluids #################### //
// ######################################################################### //

#if 0
static __device__ float P(float x) {
	return 3.2f/(x*x*x*x);
}

static __device__ float smoothingKernel(vec3 difference, float kernelWidth, float sigma, int d) {
	float result = sigma / quickPow(kernelWidth, d) * P(difference.norm()/kernelWidth);
	return result;
}

static __device__ float reconstructDensity(ParticlePositionRadius particlePosition, int numNeighbors, int firstNeighbor, ArrayView<ParticlePositionRadius> positions, ArrayView<ParticleMomentumMass> moms, float kernelWidth) {
	float result = 0.f;
	for(int i = 0; i < numNeighbors; ++i) {
		ParticlePositionRadius neighborPos;
		Saiga::CUDA::vectorCopy(positions.data()+firstNeighbor+i, &neighborPos);

		ParticleMomentumMass neighborMom;
		Saiga::CUDA::vectorCopy(moms.data()+firstNeighbor+i, &neighborMom);

		float intermediate = (1.f/neighborMom.massinv) * smoothingKernel(neighborPos.position - particlePosition.position, kernelWidth);
		result += intermediate;
	}
	return result;
}

/**
 *	k and gamma are stiffness parameters
 *	restDensity is also known as ro0
 */
static __device__ float reconstructPressure(float k, int gamma, float restDensity, float density) {
	float pressure = 0.f;
	pressure = k * restDensity * (quickPow(density/restDensity, gamma) - 1.f);
	return pressure;
}
#endif
