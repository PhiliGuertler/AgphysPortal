export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:/soft/devtools/agphys:$LD_LIBRARY_PATH
export CUDADIR=/usr/local/cuda
export CUDACXX=/usr/local/cuda/bin/nvcc


export CXX=g++-8
rm -rf build
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="/soft/devtools/agphys/" ..

#cd ..
#rm -rf build_debug
#mkdir build_debug
#cd build_debug
#cmake -DCMAKE_PREFIX_PATH="/soft/devtools/agphys/" -DCUDA_DEBUG=ON ..
