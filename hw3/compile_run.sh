mkdir -p ./build
pushd ./build
cmake ..
make -j4
echo "***************************************************************************************"
# ./Rasterizer output.png normal
# ./Rasterizer output.png phong
./Rasterizer output.png texture
popd
