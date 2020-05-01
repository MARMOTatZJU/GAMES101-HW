mkdir -p ./build
pushd ./build
cmake ..
make
echo "***************************************************************************************"
./ropesim -s 65536
popd
