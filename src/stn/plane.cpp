#include "stn/plane.h"

namespace dso {

planePoint::planePoint(Vec3 pt3) {
  ph = NULL;
  pnStruct = NULL;
  pt = pt3;
};

bool plane::addPlanarPoint(planePoint *ptp) {
  if (isPtGood(ptp->pt)) {
    updateRes(ptp->pt);
    ptSet.push_back(ptp);
    ptNum++;
  }
};
double plane::distance(const Eigen::MatrixXd data) {
  assert(data.cols() == 3);
  return ((data - centroid) * normvec).sum();
}
// plane::plane(const plane &other) { return plane(other.normv, other.centroid);
// }
// plane &plane::operator=(const dso::plane &other) { return plane(other); }

void plane::best_plane_from_points(const std::vector<Vec3> &c) {
  // copy coordinates to  matrix in Eigen format
  size_t num_atoms = c.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
  for (size_t i = 0; i < num_atoms; ++i)
    coord.col(i) = c[i];

  // calculate centroid
  best_plane_from_points(coord);
}

void plane::best_plane_from_points(const std::vector<planePoint> &c) {
  // copy coordinates to  matrix in Eigen format
  size_t num_atoms = c.size();
  assert(num_atoms > 3);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> coord(3, num_atoms);
  for (size_t i = 0; i < num_atoms; ++i)
    coord.col(i) = c[i].pt;

  best_plane_from_points(coord);
}

void plane::best_plane_from_points(
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &coord) {

  // calculate centroid
  Vec3 centr(coord.row(0).mean(), coord.row(1).mean(), coord.row(2).mean());

  // subtract centr
  coord.row(0).array() -= centr(0);
  coord.row(1).array() -= centr(1);
  coord.row(2).array() -= centr(2);

  // we only need the left-singular matrix here
  //  http://math.stackexchange.com/questions/99299/best-fitting-plane-given-a-set-of-points
  auto svd = coord.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
  Vec3 plane_normal = svd.matrixU().rightCols<1>();
  normvec = plane_normal;
  centroid = centr;
}

std::pair<Vec3, Vec3> plane::best_line_from_points(const std::vector<Vec3> &c) {
  // copy coordinates to  matrix in Eigen format
  size_t num_atoms = c.size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> centers(num_atoms, 3);
  for (size_t i = 0; i < num_atoms; ++i)
    centers.row(i) = c[i];

  Vec3 origin = centers.colwise().mean();
  Eigen::MatrixXd centered = centers.rowwise() - origin.transpose();
  Eigen::MatrixXd cov = centered.adjoint() * centered;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
  Vec3 axis = eig.eigenvectors().col(2).normalized();

  return std::make_pair(origin, axis);
}
void plane::EigenMatrix2PCLPoint(const Eigen::MatrixXd &em,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  assert(em.rows() > 0 && em.cols() == 3);
  cloud->width = em.rows();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);
  for (size_t i = 0; i < em.rows(); i++) {
    cloud->points[i].x = em(i, 0);
    cloud->points[i].y = em(i, 1);
    cloud->points[i].z = em(i, 2);
  }
}
void plane::seg_init(float th, int method) {
  sacmodel_types = method;
  cloud =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

  coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
  inliers = pcl::PointIndices::Ptr(new pcl::PointIndices);
  // Create the segmentation object

  // Optional
  if (sacmodel_types == pcl::SACMODEL_NORMAL_PLANE) {
  }

  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(th);

  // seg.setInputCloud(cloud);
  // seg.segment(*inliers, *coefficients);
}
void plane::run_seg() {
  seg.setInputCloud(cloud);

  seg.segment(*inliers, *coefficients);
  assert(inliers->indices.size() == 0);
}
int plane::readPLY(const std::string &filename) {
  if (pcl::io::loadPLYFile(filename.c_str(), *cloud) < 0) {
    std::cout << "Error loading point cloud " << filename << std::endl
              << std::endl;
    return -1;
  }
  return 0;
}

int plane::readPCD(const std::string &filename) {
  if (pcl::io::loadPCDFile(filename.c_str(), *cloud) < 0) {
    std::cout << "Error loading point cloud " << filename << std::endl
              << std::endl;
    // showHelp(argv[0]);
    return -1;
  }
  return 0;
}

int plane::matrix_transform(const Eigen::Affine3f &T) {
  // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(
  //     new pcl::PointCloud<pcl::PointXYZ>());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud(*cloud, *cloud, T);
  return 0;
}

void plane::pcl_view(const std::string &viewer_str) {
  pcl::visualization::PCLVisualizer viewer(viewer_str.c_str());

  // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
      source_cloud_color_handler(cloud, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud(cloud, source_cloud_color_handler, "original_cloud");

  viewer.addCoordinateSystem(1.0, 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05,
                            0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  // viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped()) {
    // Display the visualiser until 'q' key is pressed
    viewer.spinOnce();
  }
}
int plane::segment() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // Fill in the cloud data
  cloud->width = 15;
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  // Generate the data
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1.0 + rand() / (RAND_MAX + 1.0f) * 0.5;
  }

  // Set a few outliers
  cloud->points[0].z = 2.0;
  cloud->points[3].z = -2.0;
  cloud->points[6].z = 4.0;

  std::cerr << "Point cloud data: " << cloud->points.size() << " points"
            << std::endl;
  for (size_t i = 0; i < cloud->points.size(); ++i)
    std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y
              << " " << cloud->points[i].z << std::endl;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.3);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0) {
    PCL_ERROR("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
            << coefficients->values[1] << " " << coefficients->values[2] << " "
            << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
  for (size_t i = 0; i < inliers->indices.size(); ++i)
    std::cerr << inliers->indices[i] << "    "
              << cloud->points[inliers->indices[i]].x << " "
              << cloud->points[inliers->indices[i]].y << " "
              << cloud->points[inliers->indices[i]].z << std::endl;

  return 0;
}
}
