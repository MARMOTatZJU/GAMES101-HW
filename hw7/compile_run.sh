mkdir -p ./build
pushd ./build
cmake -DCMAKE_BUILD_TYPE=RELEASE ..
make
echo "***************************************************************************************"
./RayTracing
popd
