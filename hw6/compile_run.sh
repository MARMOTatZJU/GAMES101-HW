mkdir -p ./build
pushd ./build
cmake ..
make
echo "***************************************************************************************"
./RayTracing
popd
