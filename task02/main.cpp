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
  auto c = area(org, ps, pe);
  auto d = area(dir+ps, ps, pe);
  if (a * b > 0.f && d * c < 0.f) { return 1; }
  return 0;
  // the following code was a bug
  //auto d = area(org + dir, ps, pe);
  //if (a * b > 0.f && d * c > 0.f && fabs(d) > fabs(c)) { return 1; }
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
  // return number_of_intersection_ray_against_edge(org, dir, ps, pe);
  // write some code below to find the intersection between ray and the quadratic
  // Bézier curve defined by ps, pc, and pe.
  const auto p = [&ps, &pc, &pe](float t)
  {
    return (1 - t) * (1 - t) * ps + 2 * (1 - t) * t * pc + t * t * pe;
  };
  const auto p_prime = [&ps, &pc, &pe](float t)
  {
    return 2 * (1 - t) * (pc - ps) + 2 * t * (pe - pc);
  };
  const auto p_double_prime = 2 * (pe - 2 * pc + ps);
  const auto w = Eigen::Vector2f(-dir[1], dir[0]);
  const auto f0 = [&org, &p, &w](float t)
  {
    return (p(t) - org).dot(w);
  };
  const auto f1 = [&p_prime, &w](float t)
  {
    return p_prime(t).dot(w);
  };
  const float f1_prime = p_double_prime.dot(w);
  const float a = f1_prime / 2;
  const float b = f1(0);
  const float c = f0(0);

  // method 1. solve the quadratic equation directly
  if (b * b - 4 * a * c < 0) { return 0; }
  else if (b * b - 4 * a * c == 0) { 
    const float t = -b / (2 * a);
    int res = 0;
    if (0 < t && t < 1)
    {
      const float s = (p(t) - org)[0] / dir[0];
      if (s > 0) {
        res++;
      }
    }
    return res;
  }
  else { 
    const float t0 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    const float t1 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    int res = 0;
    if (0 < t0 && t0 < 1)
    {
      const float s = (p(t0) - org)[0] / dir[0];
      if (s > 0) {
        res++;
      }
    }
    if (0 < t1 && t1 < 1)
    {
      const float s = (p(t1) - org)[0] / dir[0];
      if (s > 0) {
        res++;
      }
    }
    return res;
  }
  
  // method 2. solve the quadratic equation using sturm's method and bisection method
  // const float d = 2 * a;
  // const float e = b;
  // const float f2 = -(c - b * e / d + a * e * e / (d * d));
  // const auto count_sign_changes = [&f0, &f1, &f2](float t)
  // {
  //   int count = 0;
  //   const float f0_t = f0(t);
  //   const float f1_t = f1(t);
  //   const float f2_t = f2;
  //   if ((f0_t > 0 && f1_t <= 0) || (f0_t < 0 && f1_t >= 0)) { count++; }
  //   if ((f1_t > 0 && f2_t <= 0) || (f1_t < 0 && f2_t >= 0)) { count++; }
  //   return count;
  // };
  // float t_min = 0;
  // float t_max = 1;
  // int num_roots = count_sign_changes(t_min) - count_sign_changes(t_max);
  // if (num_roots == 1)
  // {
  //   float t_mid = (t_min + t_max) / 2;
  //   while (t_max - t_min > 1e-6) {
  //     t_mid = (t_min + t_max) / 2;
  //     if (count_sign_changes(t_min) - count_sign_changes(t_mid) == 1) { t_max = t_mid; }
  //     else if (count_sign_changes(t_mid) - count_sign_changes(t_max) == 1) { t_min = t_mid; }
  //     else { break; }
  //   }
  //   const float s = (p(t_mid) - org)[0] / dir[0];
  //   if (s > 0) {
  //     return 1;
  //   }
  // }
  // else if (num_roots == 2) {
  //   float t_min0 = t_min;
  //   float t_max0 = - b / (2 * a);
  //   float t_min1 = - b / (2 * a);
  //   float t_max1 = t_max;
  //   float t_mid0 = (t_min0 + t_max0) / 2;
  //   float t_mid1 = (t_min1 + t_max1) / 2;
  //   while (t_max0 - t_min0 > 1e-6) {
  //     t_mid0 = (t_min0 + t_max0) / 2;
  //     if (count_sign_changes(t_min0) - count_sign_changes(t_mid0) == 1) { t_max0 = t_mid0; }
  //     else if (count_sign_changes(t_mid0) - count_sign_changes(t_max0) == 1) { t_min0 = t_mid0; }
  //     else { break; }
  //   }
  //   while (t_max1 - t_min1 > 1e-6) {
  //     t_mid1 = (t_min1 + t_max1) / 2;
  //     if (count_sign_changes(t_min1) - count_sign_changes(t_mid1) == 1) { t_max1 = t_mid1; }
  //     else if (count_sign_changes(t_mid1) - count_sign_changes(t_max1) == 1) { t_min1 = t_mid1; }
  //     else { break; }
  //   }
  //   int res = 0;
  //   float s = (p(t_mid0) - org)[0] / dir[0];
  //   if (s > 0) {
  //     res++;
  //   }
  //   s = (p(t_mid1) - org)[0] / dir[0];
  //   if (s > 0) {
  //     res++;
  //   }
  //   return res;
  // }
  // return 0;
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
      const auto dir = Eigen::Vector2f(60., 20.); // search direction
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
