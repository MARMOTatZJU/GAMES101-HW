mkdir -p ./build
pushd ./build
cmake ..
make
echo "***************************************************************************************"
./ropesim
popd
