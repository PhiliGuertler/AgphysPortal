export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:/soft/devtools/agphys:$LD_LIBRARY_PATH
export CUDADIR=/usr/local/cuda-10.2
export CUDACXX=/usr/local/cuda-10.2/bin/nvcc

export CXX=g++-8
rm -rf build
mkdir build
cd build
cmake ..
