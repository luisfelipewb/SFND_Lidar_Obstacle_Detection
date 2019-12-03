/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar * lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud = lidar->scan();
    //renderRays(viewer, lidar->position, scanCloud);
    renderPointCloud(viewer, scanCloud, "scanCloud");
    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ>  pclProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pclProcessor.SegmentPlane(scanCloud, 100, 0.2);
    //renderPointCloud(viewer, segmentCloud.first, "obstaclesCloud", Color(1,0,0));
    //renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0,1,0));

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = pclProcessor.Clustering(segmentCloud.first, 1.5, 3, 200);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster: clusters){
      std::cout << "cluster size :";
      pclProcessor.numPoints(cluster);
      renderPointCloud(viewer, cluster, "obstacle"+std::to_string(clusterId), colors[clusterId%colors.size()]);

      Box box = pclProcessor.BoundingBox(cluster);
      renderBox(viewer, box, clusterId);

      ++clusterId;
    }


}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------


  std::cout << "Number of points before filtering: " << inputCloud->width *inputCloud->height << std::endl;

  //renderPointCloud(viewer,inputCloud,"inputCloud");

  pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.1f, Eigen::Vector4f (-20, -5, -2, 1), Eigen::Vector4f (40, 7, 5, 1) );

  std::cout << "Number of points after filtering: " << filteredCloud->width *filteredCloud->height << std::endl;

  //renderPointCloud(viewer,filteredCloud,"filteredCloud");

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->NewSegmentPlane(filteredCloud, 100, 0.2);
  //renderPointCloud(viewer, segmentCloud.first, "obstaclesCloud", Color(1,0,0));
  //renderPointCloud(viewer, segmentCloud.second, "roadCloud", Color(0,1,0));

  std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessorI->NewClustering(segmentCloud.first, 0.4, 25, 2000);

  int clusterId = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster: clusters){
    std::cout << "cluster size :";
    pointProcessorI->numPoints(cluster);
    renderPointCloud(viewer, cluster, "obstacle"+std::to_string(clusterId), colors[clusterId%colors.size()]);

    Box box = pointProcessorI->BoundingBox(cluster);
    renderBox(viewer, box, clusterId);

    ++clusterId;
  }

  // Car box
  Box box;
  box.x_min = -1.8;
  box.y_min = -1.8;
  box.z_min = -1;
  box.x_max = 2.8;
  box.y_max = 1.8;
  box.z_max = 0;
  //renderBox(viewer, box, ++clusterId);
  // Road box
  box.x_min = -20;
  box.y_min = -5;
  box.z_min = -2;
  box.x_max = 40;
  box.y_max = 7;
  box.z_max = 5;
  //renderBox(viewer, box, ++clusterId);
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
      streamIterator = stream.begin();
      viewer->spinOnce ();
    }
}
