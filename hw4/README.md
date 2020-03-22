
## Description of funciton

* _recursive_bezier_: draw bezier recursively, left_bezier_point * (1-t) + right_bezier_point * t
  * left_bezier = control_points[:-1], 
  * right_bezier = control_points[1:],   
* _bezier_: high-level function that call recursive version

## De Casteljau

See _my_bezier_curve1.png_ - _my_bezier_curve6.png_ under _/images_ folder.

* _my_bezier_curve1.png_ & _my_bezier_curve1.png_: regular results
* _my_bezier_curve3.png_ & _my_bezier_curve4.png_: results with more control points
* _my_bezier_curve5.png_: result with bug fixed (cv::Point2i -> cv::Point2f)

## Antialiasing

See _my_bezier_curve-antialiasing.png_ under _/images_ folder.

Draw the adjacent grid pixel w.r.t. the distance between the precise pixel on the curve & grid pixel.


