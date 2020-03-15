Result w/t MSAA is stored in [image.png](./image.png).
Bonus has been finished. Please check out its result in [image_msaa.png](./image_msaa.png).

In _rasterizer.cpp_:
* isSameSide: test if points are located at the same side w.r.t. a line
* insideTriangle: test if a point is inside a triangle
* rasterize_triangle: rasterize a triangle by using MSAA
* depth_buf_msaa2x2: depth buffer for MSAA
* mix_pixel: added for MSAA

For effect w/o MSAA: 
* uncomment L180-191 and comment L194-222

For effect w/t MSAA: 
* comment L180-191 and uncomment L194-222

### Personal notes

#### Black edge issue in MSAA

Need to consider visibility of each super pixel _independently_.

Need to _merge_ these super pixel (mix_pixel).
