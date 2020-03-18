mkdir -p ./build
pushd ./build
cmake ..
make
echo "***************************************************************************************"
./BezierCurve
popd
