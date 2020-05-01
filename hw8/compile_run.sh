mkdir -p ./build
pushd ./build
cmake ..
make
echo "***************************************************************************************"
./ropesim -s 8192
popd
