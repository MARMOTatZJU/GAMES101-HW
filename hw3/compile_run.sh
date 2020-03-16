mkdir -p ./build
pushd ./build
cmake ..
make -j4
echo "***************************************************************************************"
./Rasterizer
popd
