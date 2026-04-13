#include <filesystem>
#include <iostream>
#include <optional>
#include <vector>

#include "Eigen/Dense"
#include "nlohmann/json.hpp"
#include "opencv2/opencv.hpp"

#include "ply_stream.hpp"
#include "view.hpp"
#include "mask.hpp"


// this defined the ground-plane for this dataset.
const Eigen::Vector3f ground_plane_point = Eigen::Vector3f(-112.66657257080078, -117.26902770996094, 29.7415714263916);
const Eigen::Vector3f ground_plane_normal = Eigen::Vector3f(-0.009972016332064267, 0.001650051087892894, -0.9999489168060939);

int main() {

  // load views from json file:
  std::vector<View> views = loadViewsFromJson(std::string(PROJECT_ROOT) + "/data/views.json");
  std::cout << "Loaded " << views.size() << " views" << std::endl;

  // example: print the first 3 views:
  for (int i = 0; i < 3; i++) {
    std::cout << "--------------------------------" << std::endl;
    std::cout << "View " << i << ": " << views[i].filename << std::endl;
    std::cout << "Intrinsic: " << views[i].intrinsic.K << std::endl;
    std::cout << "Image size: " << views[i].intrinsic.rows << "x" << views[i].intrinsic.cols << std::endl;
    std::cout << "Extrinsic: " << views[i].extrinsic.R << std::endl;
    std::cout << "Center: " << views[i].extrinsic.c.transpose() << std::endl;
    std::cout << "--------------------------------" << std::endl;
  }

  // example: load and print the mask for the first view:
  cv::Mat mask = loadMask(std::string(PROJECT_ROOT) + "/data/masks", views[0]);
  std::cout << "--------------------------------" << std::endl;
  std::cout << "loaded mask for view " << views[0].filename << " with size " << mask.size() << std::endl;
  std::cout << "number of non-zero pixels in mask: " << cv::countNonZero(mask) << std::endl;
  std::cout << "--------------------------------" << std::endl;

  // example: extract the boundary of the mask for the first view:
  MultiPolygon2D boundary = extractBoundary(mask);
  std::cout << "--------------------------------" << std::endl;
  std::cout << "Boundary of mask for view " << views[0].filename << ": " << boundary.size() << " polygons" << std::endl;
  // std::cout << bg::wkt(boundary) << std::endl;
  std::cout << "--------------------------------" << std::endl;

  // example: get the 3D ray for the central pixel in the first view:
  Ray ray = views[0].pixelToRay(views[0].intrinsic.cols / 2, views[0].intrinsic.rows / 2);
  std::cout << "--------------------------------" << std::endl;
  std::cout << "3D ray for view " << views[0].filename << " at central pixel: " << std::endl;
  std::cout << "3D ray for central pixel: " << ray.direction.transpose() << std::endl;
  std::cout << "3D ray origin: " << ray.origin.transpose() << std::endl;
  std::cout << "--------------------------------" << std::endl;

  // // example: write a few points and line segments to a ply file:
  // PointPlyStream ply_stream(std::string(PROJECT_ROOT) + "/output/pt_cloud.ply");
  // ply_stream.WriteHeader({"float x", "float y", "float z", "uchar red", "uchar green", "uchar blue"});
  // ply_stream << 0.f << 0.f << 0.f << (unsigned char)255 << (unsigned char)0 << (unsigned char)0;
  // ply_stream << 100.f << 0.f << 0.f << (unsigned char)255 << (unsigned char)0 << (unsigned char)0;
  // ply_stream << 100.f << 100.f << 0.f << (unsigned char)255 << (unsigned char)0 << (unsigned char)0;
  // ply_stream << 0.f << 100.f << 0.f << (unsigned char)255 << (unsigned char)0 << (unsigned char)0;
  // generate3DLineSegment({5, 5, 0}, Eigen::Vector3f(95, 95, 0), 1.f, 0, 255, 0, ply_stream);
  // generate3DLineSegment({50, 50, 0}, Eigen::Vector3f(50,50,100), 1.f, 0, 0, 255, ply_stream);  
  
  

  // path to the folder containing all mask .png files
  std::string mask_folder = std::string(PROJECT_ROOT) + "/data/masks";

  // -------------------------------------------------------------------------
  // PHASE 1: Load and clean masks
  // -------------------------------------------------------------------------

  // list of (view, boundary) pairs — only views with a non-empty mask end up here
  std::vector<std::pair<View, MultiPolygon2D>> view_boundaries;

  // iterating over every camera view loaded from views.json
  for (const auto &view : views) {

    // load the grayscale mask image for this view, resized to match camera resolution
    cv::Mat mask = loadMask(mask_folder, view);

    // build a 3x3 rectangular kernel (a small block of pixels used as a brush)
    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    // OPEN = erode then dilate — shrinks blobs then grows them back, killing tiny noise dots
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel3);

    // build a slightly larger 5x5 kernel
    cv::Mat kernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // CLOSE = dilate then erode — grows blobs then shrinks them back, filling small interior holes
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel5);

    // if no white pixels remain after cleanup, this view has no detection — skip it
    if (cv::countNonZero(mask) == 0) continue;

    // trace the outline of every white region into a list of 2D polygons (pixel coords)
    MultiPolygon2D boundary = extractBoundary(mask);

    // save this view paired with its pixel-space boundary for use in phase 2
    view_boundaries.push_back({view, boundary});
  }

  std::cout << "Phase 1 done: " << view_boundaries.size() << " views with valid masks" << std::endl;

  // -------------------------------------------------------------------------
  // PHASE 2: Backproject each mask boundary onto the ground plane
  // -------------------------------------------------------------------------

  // lambda: cast a ray into the scene and return where it hits the ground plane (if it does)
  // returns empty (nullopt) if the ray is parallel to the plane or points away from it
  auto rayPlaneIntersect = [&](const Ray &ray) -> std::optional<Eigen::Vector3f> {
    // dot product tells us how much the ray direction aligns with the plane normal
    float denom = ray.direction.dot(ground_plane_normal);
    // near-zero means the ray travels almost parallel to the plane — no intersection
    if (std::abs(denom) < 1e-6f) return std::nullopt;

    // t is the distance along the ray to the intersection point
    float t = (ground_plane_point - ray.origin).dot(ground_plane_normal) / denom;
    // negative t means the plane is behind the camera
    if (t < 0) return std::nullopt;

    // walk t units along the ray from the camera center to get the 3D world point
    return ray.origin + t * ray.direction;
  };

  // lambda: given a world XY position, solve for Z on the ground plane
  // derived from: dot((x,y,z) - plane_point, plane_normal) = 0, solved for z
  auto worldZ = [&](float x, float y) -> float {
    return ground_plane_point.z()
           - (ground_plane_normal.x() * (x - ground_plane_point.x())
              + ground_plane_normal.y() * (y - ground_plane_point.y()))
             / ground_plane_normal.z();
  };

  // will hold one projected polygon collection per view, in world XY coordinates
  std::vector<MultiPolygon2D> projected_boundaries;

  // iterating over each (view, pixel boundary) pair from phase 1
  for (const auto &[view, pixel_boundary] : view_boundaries) {

    // accumulates all projected polygons for this single view
    MultiPolygon2D view_projected;

    // iterating over each separate polygon blob in this view's pixel-space boundary
    for (const auto &pixel_poly : pixel_boundary) {

      // empty polygon that will be filled with world-space vertices
      Polygon2D world_poly;

      // iterating over each vertex on the outer ring of this pixel polygon
      for (const auto &px_pt : pixel_poly.outer()) {
        float u = bg::get<0>(px_pt); // column (x) in pixel space
        float v = bg::get<1>(px_pt); // row    (y) in pixel space

        // build the 3D ray that passes through this pixel from the camera
        Ray ray = view.pixelToRay(u, v);

        // find where that ray hits the ground plane
        auto world_pt = rayPlaneIntersect(ray);

        // if the ray missed the plane (parallel or behind camera), skip this vertex
        if (!world_pt) continue;

        // append the world XY position — Z will be reconstructed later from the plane
        bg::append(world_poly.outer(), Point2D(world_pt->x(), world_pt->y()));
      }

      // need at least 3 points to form a valid polygon
      if (world_poly.outer().size() < 3) continue;

      // close the ring by repeating the first point at the end (Boost Geometry requirement)
      bg::append(world_poly.outer(), world_poly.outer().front());

      // fix vertex winding order (Boost expects counter-clockwise for outer rings)
      bg::correct(world_poly);

      // add this projected polygon to this view's collection
      view_projected.push_back(world_poly);
    }

    // store all projected polygons for this view
    projected_boundaries.push_back(view_projected);
  }

  std::cout << "Phase 2 done: backprojected " << projected_boundaries.size() << " views" << std::endl;

  // -------------------------------------------------------------------------
  // PHASE 3: Union all projected polygons into a single world-space mask
  // -------------------------------------------------------------------------

  // starts empty; each view's polygons are merged into this one by one
  MultiPolygon2D accumulator;

  // iterating over each view's projected polygon collection
  for (const auto &view_mp : projected_boundaries) {
    // iterating over each individual polygon in this view
    for (auto poly : view_mp) {
      // re-check winding order before merging
      bg::correct(poly);

      // merge this polygon into the accumulator — overlapping regions become one solid shape
      MultiPolygon2D result;
      bg::union_(accumulator, poly, result);
      accumulator = result; // result becomes the new accumulator for the next iteration
    }
  }

  // remove redundant vertices that deviate less than 0.1 world units from the true outline
  MultiPolygon2D final_mask;
  bg::simplify(accumulator, final_mask, 0.1);

  std::cout << "Phase 3 done: union has " << final_mask.size() << " polygon(s)" << std::endl;

  // -------------------------------------------------------------------------
  // PHASE 4: Write the world-coordinate mask boundary to a PLY file
  // -------------------------------------------------------------------------

  // create the output folder if it doesn't already exist
  std::filesystem::create_directories(std::string(PROJECT_ROOT) + "/outputs");

  // open the output PLY file for writing
  std::string out_path = std::string(PROJECT_ROOT) + "/outputs/world_mask_boundary.ply";
  PointPlyStream ply_stream(out_path);

  // declare the per-point data layout: 3D position + RGB color
  ply_stream.WriteHeader({"float x", "float y", "float z", "uchar red", "uchar green", "uchar blue"});

  // running count of boundary vertices across all polygons (for the summary print)
  int total_vertices = 0;

  // iterating over each polygon in the final merged mask
  for (const auto &poly : final_mask) {
    // get the outer boundary ring (list of 2D world XY vertices)
    const auto &ring = poly.outer();

    // iterating over each consecutive edge: vertex i -> vertex i+1
    for (std::size_t i = 0; i + 1 < ring.size(); ++i) {
      // extract XY for both endpoints of this edge
      float ax = bg::get<0>(ring[i]),   ay = bg::get<1>(ring[i]);
      float bx = bg::get<0>(ring[i+1]), by = bg::get<1>(ring[i+1]);

      // lift each 2D world point back to 3D by computing Z from the plane equation
      Eigen::Vector3f pt_a(ax, ay, worldZ(ax, ay));
      Eigen::Vector3f pt_b(bx, by, worldZ(bx, by));

      // fill the edge with a point every 0.5 world units, colored green
      generate3DLineSegment(pt_a, pt_b, 0.5f,
                            /*r*/0, /*g*/255, /*b*/0,
                            ply_stream);
    }

    // accumulate vertex count for the summary
    total_vertices += static_cast<int>(ring.size());
  }

  std::cout << "Phase 4 done: wrote " << final_mask.size() << " polygon(s), "
            << total_vertices << " boundary vertices -> " << out_path << std::endl;

  return 0;
}
