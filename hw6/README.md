
I've finished task:
* [5 points] 提交格式正确,包含所有需要的文件;代码可以在虚拟机下正确编译运行。
* [20 points] 包围盒求交:正确实现光线与包围盒求交函数
* BVH 查找:正确实现 BVH 加速的光线与场景求交。

Result: /image/bvh.ppm

Description of function
* Bounds3::IntersectP
  * intersection with AABB
* BVHAccel::getIntersection
  * BVH query algorithm
  * query the intersection of a ray with a prebuilt BVH
