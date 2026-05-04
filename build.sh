mkdir -p build
cd build
cmake ..
#cmake .. -DBUILD_TESTS=ON
#cmake .. -DBUILD_DEPTHAI_APP=ON
make -j4