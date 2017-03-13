#include <pcl/filters/passthrough.h>
/***
点的目的是
1. 转化格式，需要从原始的地图中的点，转化格式为pcl的格式
2. 给点filter，减少计算量
3. 用于显示

***/
namespace dso {
class point {
public:
  point(const pcl::PointCloud<PointXYZ>::Ptr cloud);
  ptsFilter(const pcl::PointCloud<PointXYZ>::Ptr cloud);

private:
  pcl::PointCloud<PointXYZ>::Ptr cloud;
  pcl::PointCloud<PointXYZ>::Ptr filtedCloud;
  pcl::
}
}
