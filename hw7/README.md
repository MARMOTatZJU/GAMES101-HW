
I've finished task ...

* Path Tracing
  * Scene.cpp, Scene::castRay: recursive tracing with Russian Roulette
  * diffuse-SPP16.ppm, diffuse-SPP128.ppm, diffuse-SPP512.ppm
* 多线程
  * Render.cpp, with openmp
  * 400minute for SPP=512
* Microfacet (partially done)
  * microfacet-SPP1.ppm, microfacet-SPP4.ppm
    * microfacet-SPP8.ppm will have problem of "over-exposed"
  * Material.hpp
  * Material::eval_microfacet