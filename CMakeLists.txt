cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

if(CUDA_DEBUG)
	set(PROG_NAME agphys_debug)
else()
	set(PROG_NAME agphys)
endif()
project (${PROG_NAME} VERSION 1.0.0 LANGUAGES CXX CUDA)


#To make sure that the host code also sees the cuda headers.
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH} )
include(helper_macros)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)


DefaultBuildType(RelWithDebInfo)

############# Required LIBRARIES ###############


message("\nSearching for installed libaries...")

#saiga
find_package(SAIGA REQUIRED QUIET)
PackageHelperTarget(Saiga::saiga_core SAIGA_FOUND)
#PackageHelperTarget(Saiga::saiga_cuda SAIGA_FOUND)
PackageHelperTarget(Saiga::saiga_opengl SAIGA_FOUND)

#eigen
find_package(Eigen3 REQUIRED)
PackageHelperTarget(Eigen3::Eigen EIGEN_FOUND)

#c++17 filesystem
find_package(Filesystem REQUIRED)
#PackageHelperTarget(std::filesystem FILESYSTEM_FOUND)

#openmp
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -g")
endif()
PackageHelper(OpenMP "${OPENMP_FOUND}" "" "")

message("")

############# COMPILER FLAGS ###############

OPTION(CUDA_COMPATIBILITY_MODE "Add CUDA compile flags that make the code compatible with older GPUs but takes longer to compile" OFF)
OPTION(CUDA_PROFILING "Enables the PROFILING #define" OFF)
OPTION(CUDA_DEBUG "Enables CUDA Debugging in Nsight" OFF)
OPTION(FULL_SPEED "Enables All Optimizations" OFF)

if (CUDA_COMPATIBILITY_MODE) 
       message("Compatibility Mode: -gencode=arch=compute_30,code=compute_30")
       SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=compute_30,code=compute_30")
else()
	message("No Compatibility")
endif()

if(CUDA_PROFILING)
        add_definitions(-DCUDA_PROFILING)
        SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -lineinfo")
        SET(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

if(CUDA_DEBUG)
        SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -G")
        SET(CMAKE_BUILD_TYPE DEBUG)
endif()

if(FULL_SPEED)
        add_definitions(-DCUDA_NDEBUG)
        SET(CMAKE_BUILD_TYPE RELEASE)
    else()
        add_definitions(-DCUDA_DEBUG)
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall")
SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -march=native")



# find cuda architecture
#Get CUDA compute capability
set(OUTPUTFILE ${CMAKE_CURRENT_SOURCE_DIR}/cuda_script) # No suffix required
set(CUDAFILE ${CMAKE_CURRENT_SOURCE_DIR}/check_cuda.cu)
execute_process(COMMAND nvcc -lcuda ${CUDAFILE} -o ${OUTPUTFILE})
execute_process(COMMAND ${OUTPUTFILE}
	RESULT_VARIABLE CUDA_RETURN_CODE
	OUTPUT_VARIABLE ARCH)

if(${CUDA_RETURN_CODE} EQUAL 0)
	set(CUDA_SUCCESS "TRUE")
else()
	set(CUDA_SUCCESS "FALSE")
endif()

if (${CUDA_SUCCESS})
	message(STATUS "CUDA Architecture: ${ARCH}")

	set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} ${ARCH}")
else()
	message(WARNING ${ARCH})
endif()




#SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode arch=compute_52,code=sm_52")

#if(${CMAKE_CUDA_COMPILER_VERSION} VERSION_GREATER_EQUAL "9")
#SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=compute_61,code=sm_61") # Pascal
#endif()

#SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=compute_70,code=sm_70") # Volta


#if(${CMAKE_CUDA_COMPILER_VERSION} VERSION_GREATER_EQUAL "10")
#SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -gencode=arch=compute_75,code=sm_75") # Turing
#endif()

SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -use_fast_math --expt-relaxed-constexpr")






SET(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcudafe --diag_suppress=esa_on_defaulted_function_ignored")

message(STATUS "CUDA FLAGS: ${CMAKE_CUDA_FLAGS}")
#SET(CMAKE_LD_FLAGS "${CMAKE_LD_FLAGS} -lcusparsea")


if(MSVC)
	#multiprocessor compilation for visual studio
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") 
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-strict-aliasing")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")
        if(FULL_SPEED)
            SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
        endif()
endif()




############# Generate config file ###############

set(BUILDCONFIG_DIR "${CMAKE_CURRENT_BINARY_DIR}/include")
set(BUILDCONFIG_FILE "${BUILDCONFIG_DIR}/buildconfig.h")
INCLUDE_DIRECTORIES(${BUILDCONFIG_DIR})
configure_file (
  "${CMAKE_CURRENT_SOURCE_DIR}/src/buildconfig.h.in"
  "${BUILDCONFIG_FILE}"
)



############# SOURCE ###############

add_subdirectory(src)

set_property(TARGET ${PROG_NAME} PROPERTY CXX_STANDARD 17)

#############  output dir ############# 

if(MSVC)
	#visual studio working dir
	set_target_properties(${PROG_NAME} PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()

#copies executable to top level source tree
set_target_properties( ${PROG_NAME}
	PROPERTIES
    	RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")


