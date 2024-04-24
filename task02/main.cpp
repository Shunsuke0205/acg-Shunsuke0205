#include <filesystem>
// #include <experimental/filesystem> // uncomment here if the <filesystem> cannot be included above
//
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "Eigen/Core"
//
#include "parse_svg.h"

/***
 * signed area of a triangle connecting points (p0, p1, p2) in counter-clockwise order.
 * @param p0 1st point xy-coordinate
 * @param p1 2nd point xy-coordinate
 * @param p2 3rd point xy-coordinate
 * @return signed area (float)
 */
float area(
    const Eigen::Vector2f &p0,
    const Eigen::Vector2f &p1,
    const Eigen::Vector2f &p2) {
  const auto v01 = p1 - p0;
  const auto v02 = p2 - p0;
  // return 0.5f * (v01[0] * v02[1] - v01[1] * v02[0]); // right handed coordinate
  return 0.5f * (v01[1] * v02[0] - v01[0] * v02[1]); // left-handed coordinate (because pixel y-coordinate is going down)
}


/***
 * compute number of intersection of a ray against a line segment
 * @param org ray origin
 * @param dir ray direction (unit normal)
 * @param ps one of the two end points
 * @param pe the other end point
 * @return number of intersection
 */
int number_of_intersection_ray_against_edge(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pe) {
  auto a = area(org, org + dir, ps);
  auto b = area(org, pe, org + dir);
  auto c = area(org, pe, ps);
  auto d = area(org + dir, ps, pe);
  if (a * b >= 0.f && d * c >= 0.f) {
    return 1;
  }
  // the following code was a bug
  // auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
  return 0;
}



/***
 *
 * @param org ray origin
 * @param dir ray direction (unit vector)
 * @param ps one of the two end points
 * @param pc control point
 * @param pe the other end point
 * @return the number of intersections
 */
int number_of_intersection_ray_against_quadratic_bezier(
    const Eigen::Vector2f &org,
    const Eigen::Vector2f &dir,
    const Eigen::Vector2f &ps,
    const Eigen::Vector2f &pc,
    const Eigen::Vector2f &pe) {
  // comment out below to do the assignment
  const Eigen::Vector2f dir_perpendicular(dir[1], -dir[0]); // 90 degree rotation

  std::vector<float> point_on_curve_x_coordinate_polynominal = {0.0, 0.0, 0.0};
  std::vector<float> point_on_curve_y_coordinate_polynominal = {0.0, 0.0, 0.0};

  // p(t) = (1 - t)^2 * ps + 2 * (1 - t) * t * pc + t^2 * pe
  point_on_curve_x_coordinate_polynominal[0] = ps[0]; // constant term
  point_on_curve_x_coordinate_polynominal[1] = 2 * (pc[0] - ps[0]); // linear term
  point_on_curve_x_coordinate_polynominal[2] = ps[0] - 2 * pc[0] + pe[0]; // quadratic term
  point_on_curve_y_coordinate_polynominal[0] = ps[1]; // constant term
  point_on_curve_y_coordinate_polynominal[1] = 2 * (pc[1] - ps[1]); // linear term
  point_on_curve_y_coordinate_polynominal[2] = ps[1] - 2 * pc[1] + pe[1]; // quadratic term

  // shift the origin
  point_on_curve_x_coordinate_polynominal[0] -= org[0];
  point_on_curve_y_coordinate_polynominal[0] -= org[1];

  // product of dir_perpendicular and p(t)
  std::vector<float> dir_perpendicular_dot_point_on_curve = {0.0, 0.0, 0.0};
  for (int i = 0; i < 3; i++) {
    dir_perpendicular_dot_point_on_curve[i] = point_on_curve_x_coordinate_polynominal[i] * dir_perpendicular[0] +
                                              point_on_curve_y_coordinate_polynominal[i] * dir_perpendicular[1];
  }
  
  // f(t) = a * t^2 + b * t + c
  float a = dir_perpendicular_dot_point_on_curve[2];
  float b = dir_perpendicular_dot_point_on_curve[1];
  float c = dir_perpendicular_dot_point_on_curve[0];

  float epsilon = 1e-6;
  if (abs(a) < epsilon) { // in case f(t) is linear
    if ((b + c) * c <= epsilon) {
      return 1;
    } else {
      return 0;
    }
  }

  float root_minus = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
  float root_plus  = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
  
  int number_of_roots = 0;
  if (0 <= root_minus && root_minus <= 1) {
    Eigen::Vector2f point_on_curve = (1 - root_minus) * (1 - root_minus) * ps + 2 * (1 - root_minus) * root_minus * pc + root_minus * root_minus * pe;
    if (area(org, org + dir_perpendicular, point_on_curve) < 0) {
      number_of_roots += 1;
    }
  }
  if (0 <= root_plus && root_plus <= 1) {
    Eigen::Vector2f point_on_curve = (1 - root_plus) * (1 - root_plus) * ps + 2 * (1 - root_plus) * root_plus * pc + root_plus * root_plus * pe;
    if (area(org, org + dir_perpendicular, point_on_curve) < 0) {
      number_of_roots += 1;
    }
  }

  return number_of_roots;
}

int main() {
  const auto input_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / ".." / "asset" / "r.svg";
  const auto [width, height, shape] = acg::svg_get_image_size_and_shape(input_file_path);
  if (width == 0) { // something went wrong in loading the function
    std::cout << "file open failure" << std::endl;
    abort();
  }
  const std::vector<std::string> outline_path = acg::svg_outline_path_from_shape(shape);
  const std::vector<std::vector<acg::Edge>> loops = acg::svg_loops_from_outline_path(outline_path);
  //
  std::vector<unsigned char> img_data(width * height, 255); // grayscale image initialized white
  for (unsigned int ih = 0; ih < height; ++ih) {
    for (unsigned int iw = 0; iw < width; ++iw) {
      const auto org = Eigen::Vector2f(iw + 0.5, ih + 0.5); // pixel center
      const auto dir = Eigen::Vector2f(6000., 2000.); // search direction
      int count_cross = 0;
      for (const auto &loop: loops) { // loop over loop (letter R have internal/external loops)
        for (const auto &edge: loop) { // loop over edge in the loop
          if (edge.is_bezier) { // in case the edge is a quadratic Bézier
            count_cross += number_of_intersection_ray_against_quadratic_bezier(
                org, dir,
                edge.ps, edge.pc, edge.pe);
          } else { // in case the edge is a line segment
            count_cross += number_of_intersection_ray_against_edge(
                org, dir,
                edge.ps, edge.pe);
          }
        }
      }
      if (count_cross % 2 == 1) { // Jordan's curve theory
        img_data[ih * width + iw] = 0; // paint black if it is inside
      }
    }
  }
  const auto output_file_path = std::filesystem::path(PROJECT_SOURCE_DIR) / "output.png";
  stbi_write_png(output_file_path.string().c_str(), width, height, 1, img_data.data(), width);
}
