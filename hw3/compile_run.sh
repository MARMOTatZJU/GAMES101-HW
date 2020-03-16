mkdir -p ./build
pushd ./build
cmake ..
make -j4
echo "***************************************************************************************"
# ./Rasterizer output.png normal
# ./Rasterizer output.png phong
# ./Rasterizer output.png texture
# ./Rasterizer output.png bump
./Rasterizer output.png displacement
popd
