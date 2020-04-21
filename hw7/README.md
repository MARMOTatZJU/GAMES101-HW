
I've finished task ...

* Path Tracing
  * Scene.cpp, Scene::castRay: recursive tracing with Russian Roulette
  * diffuse-SPP16.ppm, diffuse-SPP128.ppm, diffuse-SPP512.ppm, diffuse-SPP1024.ppm
* 多线程
  * Render.cpp, with openmp
    * pragma omp schedule collapse(2) dynamic for loop in Renderer.cpp
    * pragma omp critical for pbar update
    * local_thread for get_random_float in global.hpp
  * 5h for SPP=1025
* Microfacet (partially done)
  * microfacet-SPP1.ppm, microfacet-SPP4.ppm
    * microfacet-SPP8.ppm will have problem of "over-exposed"
  * Material.hpp
  * Material::eval_microfacet

  