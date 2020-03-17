# Checklist

## 提交格式正确,包括所有需要的文件。代码可以正常编译、执行。

Check it by runnning _
```
bash compile_run.sh
```

## 参数插值

_/images/spot-normal.png_

main.cpp: _normal_fragment_shader_, 
- use normal vector as color

## Blinn-phong 反射模型

_/images/spot-phong.png_

main.cpp: _phong_fragment_shader_, 
- apply same color to coefficient _kd_ based on blinn-phong reflectance model

## Texture mapping

_/images/spot-texture.png_

main.cpp: _texture_fragment_shader_, 
- query texture color on texture image
- apply texture color to coefficient _kd_ based on blinn-phong reflectance model

## Bump mapping 与 Displacement mapping

_/images/spot-bump.png_

main.cpp: _bump_fragment_shader_, 
- use TBN matrix to encode surface height in coefficient _kd_ in blinn-phong reflectance

_/images/spot-displacement.png_

main.cpp: _displacement_fragment_shader_, 
- use TBN matrix to encode surface height in coefficient _kd_ in blinn-phong reflectance, 
- make real displacement of vertices 

# Bonus

## Bonus 1

- _/images/bunny-normal.png_
- _/images/rock-texture.png_
- _/images/crate-texture.png_
- _/images/cube-texture.png_


__Nota__:Teapot & are downloaded from CSAIL@MIT under this link: https://groups.csail.mit.edu/graphics/classes/6.837/F03/models/

## Bonus 2

Comparison: 

- _/images/spot-texture.png_
- _/images/spot-texture-bilinear.png_

Texture.hpp: _getColorBilinear_, 
- use bilinear interpolation to query color at real coordinates on texture image
