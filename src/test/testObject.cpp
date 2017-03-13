
#include "stn/object.h"
#include "stn/randomGenerator.h"
#include <pangolin/pangolin.h>
using namespace dso;
using namespace std;
using namespace cv;

int main(int argc, char const *argv[]) {
  object obj(0.8);
  Vec3 center(2, 4, 6);
  Vec3 sz(10, 10, 10);
  const size_t num_pts = 1000;
  float resultC[3 * num_pts];
  for (size_t i = 0; i < num_pts; i++) {
    // Vec3 pt = randomGenerator::RandVec3Box(center, sz);
    Vec3 pt = randomGenerator::RandVec3columnar(center, 2, 6);
    obj.addPoint(pt);
    resultC[3 * i] = pt[0];
    resultC[3 * i + 1] = pt[1];
    resultC[3 * i + 2] = pt[2];
    // std::cout << pt[0] << "\t" << pt[1] << "\t" << pt[2] << "\t" <<
    // std::endl;
  }

  int window_height = 960;
  int window_width = 1280;

  int fx, fy;
  fx = fy = 100;
  int u0 = window_height / 2;
  int v0 = window_width / 2;

  pangolin::CreateWindowAndBind("Main", window_width, window_height);
  glEnable(GL_DEPTH_TEST);
  std::cout << "object points number" << obj.ptMat.cols() << '\t'
            << obj.ptMat.rows() << '\t' << obj.ptVec.size() << std::endl;

  pangolin::GlBuffer glxyz(pangolin::GlArrayBuffer, num_pts, GL_FLOAT, 3,
                           GL_STATIC_DRAW);

  glxyz.Upload(resultC, 3 * sizeof(float) * num_pts);

  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(window_width, window_height, fx, fy, u0, v0,
                                 0.2, 100),
      pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

  // Create Interactive View in window
  pangolin::Handler3D handler(s_cam);
  pangolin::View &d_cam =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -1.0 * window_width / window_width)
          .SetHandler(&handler);

  while (!pangolin::ShouldQuit()) {
    // Clear screen and activate view to render into
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    d_cam.Activate(s_cam);

    // Render OpenGL Cube
    // pangolin::glDrawColouredCube();
    Eigen::Vector3d p(-1, 0, 0);
    pangolin::glDrawCross(p, 5.0);
    // Eigen::Vector2d cir(4, 5);
    // pangolin::glDrawCircle(cir, 4);
    // pangolin::glDrawTexture(5, 1000);
    // std::cout << "/* message */" << std::endl;
    pangolin::RenderVbo(glxyz);

    // Swap frames and Process Events
    pangolin::FinishFrame();
  }

  return 0;
}
