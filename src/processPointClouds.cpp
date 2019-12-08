// PCL lib Functions for processing point clouds

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    // Remove points faraway and focus on the road
    pcl::CropBox<PointT> crop_road;
    crop_road.setInputCloud(cloud);
    crop_road.setMin(minPoint);
    crop_road.setMax(maxPoint);
    crop_road.filter(*cloud);

    // Remove ego car roof
    std::vector<int> egoCarIndices;
    pcl::CropBox<PointT> crop_car;
    crop_car.setInputCloud(cloud);
    crop_car.setMin(Eigen::Vector4f (-1.8, -1.8, -1, 1));
    crop_car.setMax(Eigen::Vector4f (2.8, 1.8, 0, 1));
    crop_car.filter(egoCarIndices);

    pcl::PointIndices::Ptr egoCarPoints (new pcl::PointIndices);
    for (int point : egoCarIndices)
      egoCarPoints->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (egoCarPoints);
    extract.setNegative (true);
    extract.filter (*cloud);

    // Downsampling to decrease resolution
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT> ());
    pcl::VoxelGrid<PointT> voxel_grid;
    voxel_grid.setInputCloud (cloud);
    voxel_grid.setLeafSize (filterRes, filterRes, filterRes);
    voxel_grid.filter (*filteredCloud);

    std::cout << "Number of points before filtering: " << cloud->size() << std::endl;
    std::cout << "Number of points after filtering: " << filteredCloud->size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << elapsedTime.count() << " ms\t - filtering"<< std::endl;

    return filteredCloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
  // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices)
        roadCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstaclesCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclesCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    // pcl::PointIndices::Ptr inliers;

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << elapsedTime.count() << " ms\t - plane segmentation" << std::endl;

    return segResult;
}


template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  while (maxIterations--) {

    std::unordered_set<int> inliersCandidate;

    // select three random points to create a plane
    while (inliersCandidate.size() < 3)
    {
      int randomIndex = rand() % cloud->points.size();
      inliersCandidate.insert(randomIndex);
    }

    float x1, y1, z1, x2, y2, z2, x3, y3, z3;
    auto iterator = inliersCandidate.begin();
    x1 = cloud->points[*iterator].x;
    y1 = cloud->points[*iterator].y;
    z1 = cloud->points[*iterator].z;
    iterator++;
    x2 = cloud->points[*iterator].x;
    y2 = cloud->points[*iterator].y;
    z2 = cloud->points[*iterator].z;
    iterator++;
    x3 = cloud->points[*iterator].x;
    y3 = cloud->points[*iterator].y;
    z3 = cloud->points[*iterator].z;

    // calculate plane equation
    float a = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
    float b = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
    float c = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);
    float d = -1 * (a*x1 + b*y1 + c*z1);

    // select all points within the distance toleration from the plane
    for (int index=0; index < cloud->points.size(); index++)
    {
      float px, py, pz;
      px = cloud->points[index].x;
      py = cloud->points[index].y;
      pz = cloud->points[index].z;

      float dist = fabs(a*px + b*py +  c*pz + d)/sqrtf(a*a+b*b+c*c);

      if (dist < distanceTol)
        inliersCandidate.insert(index);
    }

    if (inliersCandidate.size() > inliersResult.size())
      inliersResult = inliersCandidate;

  }
  return inliersResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::NewSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  auto startTime = std::chrono::steady_clock::now();

  std::unordered_set<int> inliersSet = RansacPlane(cloud, maxIterations, distanceThreshold);

  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  for (int index : inliersSet)
      inliers->indices.push_back(index);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << elapsedTime.count() << " ms\t - plane segmentation" << std::endl;

  return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Euclidean clustering to group detected obstacles
    // Uses a KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for (pcl::PointIndices getIndices: clusterIndices){
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

      for (int index: getIndices.indices)
        cloudCluster->points.push_back (cloud->points[index]);

      cloudCluster->width = cloudCluster->points.size ();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << elapsedTime.count() << " ms\t - clustering (" << clusters.size() << " clusters)" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::recursiveClustering(int index, std::vector<int>& cluster, std::vector<bool>& processed, const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);

	std::vector<int> nearby = tree->search(points[index],distanceTol);

	for(int id : nearby)
		if(!processed[id])
			recursiveClustering(id, cluster, processed, points, tree, distanceTol);

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::NewClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();

    // get points form cloud and insert into KD tree structure
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    //std::vector<pcl::PointT> cloud_data = cloud->points;

    typename pcl::PointCloud<PointT>::iterator point_it;
    int i = 0;
    for (point_it = cloud->points.begin(); point_it < cloud->points.end(); point_it++)
    {
      points.push_back({point_it->x,point_it->y,point_it->z});
      tree->insert(points[i],i);
      i++;
    }

    // Execute recursive Clustering
    std::vector<std::vector<int>> point_clusters;
    std::vector<bool> processed(points.size(), false);

    for (int id=0; id<points.size(); id++){
      if (!processed[id]){
        std::vector<int> point_cluster;
        recursiveClustering(id, point_cluster, processed, points, tree, clusterTolerance);
          if (point_cluster.size() > minSize)
            point_clusters.push_back(point_cluster);
      }
    }

    // Create cloud clusters based on cluster indices
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    for (std::vector<int> getIndices: point_clusters){
      typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

      for (int index: getIndices)
        cloudCluster->points.push_back (cloud->points[index]);

      cloudCluster->width = cloudCluster->points.size ();
      cloudCluster->height = 1;
      cloudCluster->is_dense = true;

      clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << elapsedTime.count() << " ms\t - clustering (" << clusters.size() << " clusters)" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));

    //These eigenvectors are used to transform the point cloud to the origin point (0, 0, 0) such that the eigenvectors correspond to the axes of the space. The minimum point, maximum point, and the middle of the diagonal between these two points are calculated for the transformed cloud (also referred to as the projected cloud when using PCL's PCA interface, or reference cloud by Nicola).
    // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    typename pcl::PointCloud<PointT>::Ptr cloudPointsProjected (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    // Find bounding box for one of the clusters
    BoxQ box;
    box.cube_length = maxPoint.x - minPoint.x;
    box.cube_width = maxPoint.y - minPoint.y;
    box.cube_height = maxPoint.z - minPoint.z;
    box.bboxQuaternion = eigenVectorsPCA;
    box.bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}
