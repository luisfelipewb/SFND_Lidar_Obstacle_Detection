# Project: Lidar Obstacle Detection

[//]: # (Image References)
[gif_original]: ./media/original.gif
[gif_filtered]: ./media/filtered.gif
[gif_plane]: ./media/plane.gif
[gif_clusters]: ./media/clusters.gif
[gif_result]: ./media/result.gif
[gif_data2]: ./media/data2.gif
[gif_data2_result]: ./media/data2_result.gif
[gif_data2_bonus]: ./media/data2_bonus.gif
[img_performance]: ./media/performance.png
[img_3dtree]: ./media/3dtree.png
[gif_KDTree-animation]: ./media/KDTree-animation.gif
[img_ransac_example]: ./media/gif_project_result.png


## Project Report

This project covers filtering, segmenting, and clustering real lidar point cloud data to detect vehicles and other objects.
The goal of this project is to detect other cars on the road using raw lidar data from Udacity’s real self-driving car.
For this project, a custom RANSAC and euclidean clustering algorithms were implemented.

The animation bellow shows the result of the project with red bounding boxes around the obstacles.

![][gif_result]


## Detection pipeline

### Original data

The original data comes from a Lidar sensor installed in Udacity's real autonomous vehicle, Carla.
The Lidar sensor provides 3D data with high resolution. In this project, the data type is a PCL PointXYZI format which also includes intensity information.

![][gif_original]

### Downsampling and Filtering

To reduce computational time, it is necessary remove irrelevant points. This filter basically crops the point cloud in 3D space keeping only the area of interest. For this step, the functions available in the PCL library were used.

```cpp
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
```

![][gif_filtered]


### RANSAC Plane Segmentation
Next, we need to identify and remove the surface of the road. Since the geometric shape of the ground is usually a plane, the RANSAC filter is a good option to identify it. In this project, the RANSAC algorithm was implemented and used the auxiliary function bellow.

```cpp
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
```

![][gif_plane]


### Euclidean Clustering

Next, the obstacle points must be grouped into a cluster. The clustering option used in this project is based on the euclidean distance between each point.
If the points are very close together, they are considered to belong to the same object.

The clustering was implemented with the recursive function as shown below.

```cpp
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

    return clusters;
}
```

![][gif_clusters]



### KD Tree

To optimize the clustering algorithm, a data structure known as KD Tree was used.
It organizes the point cloud data in a tree allowing to efficiently locate each point based on its position.

#### 2D KD tree illustration

![][gif_KDTree-animation]

#### 3D KD tree visulization
![][img_3dtree]

(Source: https://en.wikipedia.org/wiki/K-d_tree)

For learning purposes, a basic KD tree was implemented and the source code is [available here](./kdtree.h).



### Results

As the first animation shows, it was possible to correctly identify the main objects close to the ego car. The bounding boxes enclose the other vehicles and the pole on the right side of the ego car. The bounding boxes can be followed through the lidar stream, and major objects don't lose or gain bounding boxes in the middle of the stream.

It is worth mentioning that the code for 3D RANSAC plane segmentation and euclidean clustering were not based on the implementation available in the PCL library.
This implementation was done for learning purposes and performance was not prioritized. As the graph below shows, the functions available in the PLC library perform roughly 14 times better than the new implementation in this project.

![][img_performance]



## Conclusions and next steps

The project was a great way to get familiar with the PCL library and experiment with real Lidar sensor data.
The implementation of the algorithms also provided a good understanding of what it takes to detect objects using a lidar sensor.
Even though the performance of the algorithms was not the main concern it would be possible to improve by reviewing the implementation a bit more carefully.
Naturally, the next step would be to track the detected objects and follow them over time.

## Bonus

In the example data below, processing the data is more challenging. The car is taking turns and the types of obstacles are more diverse.
It is possible to visualize a cyclist that is correctly detected a couple of meters in front of the car. Originally, the bounding box has fixed X, Y and Z orientation using the same coordinate reference as the sensor. The main disadvantage is that the resulting bounding box can be a unnecessarily large and constrain the available space perceived by the car.

Using a statistical procedure known as Principal Component Analysis, it is possible to find a minimum oriented bounding box. The code bellow to calculate the bounding box is based on this blog post: http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html

```cpp
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
```

##### Original PCL
![][gif_data2]

##### Results
![][gif_data2_result]

##### Results with rotating minimum bounding box
![][gif_data2_bonus]
