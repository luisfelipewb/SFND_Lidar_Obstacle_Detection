/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
			for (int j=0; j<5; j++){
	  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
	  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
	  		pcl::PointXYZ point;
	  		point.x = i+scatter*rx;
	  		point.y = i+scatter*ry;
	  		point.z = 0;

	  		cloud->points.push_back(point);
			}
  	}
  	// Add outliers
  	int numOutliers = 50;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	while(maxIterations--)
	{
		std::unordered_set<int> inliersCandidate;

		while (inliersCandidate.size() < 2)
		{
			int randomIndex = rand() % cloud->points.size();
			inliersCandidate.insert(randomIndex);
		}

		auto iterator = inliersCandidate.begin();
		float x1 = cloud->points[*iterator].x;
		float y1 = cloud->points[*iterator].y;
		iterator++;
		float x2 = cloud->points[*iterator].x;
		float y2 = cloud->points[*iterator].y;

		float a = (y1-y2);
		float b = (x2-x1);
		float c = (x1*y2-x2*y1);

		for (int index=0; index < cloud->points.size(); index++){

			if(inliersCandidate.count(index))
				continue;

			float x3 = cloud->points[index].x;
			float y3 = cloud->points[index].y;

			float dist = fabs(a*x3+b*y3+c)/sqrt(a*a+b*b);

			if (dist < distanceTol){
				inliersCandidate.insert(index);
			}

		if(inliersCandidate.size() > inliersResult.size())
			inliersResult = inliersCandidate;
		}
	}
}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	// TODO: Fill in this function
	while(maxIterations--)
	{
		std::unordered_set<int> inliersCandidate;

		while (inliersCandidate.size() < 3)
		{
			int randomIndex = rand() % cloud->points.size();
			inliersCandidate.insert(randomIndex);
		}

		auto iterator = inliersCandidate.begin();
		float x1 = cloud->points[*iterator].x;
		float y1 = cloud->points[*iterator].y;
		float z1 = cloud->points[*iterator].z;
		iterator++;
		float x2 = cloud->points[*iterator].x;
		float y2 = cloud->points[*iterator].y;
		float z2 = cloud->points[*iterator].z;
		iterator++;
		float x3 = cloud->points[*iterator].x;
		float y3 = cloud->points[*iterator].x;
		float z3 = cloud->points[*iterator].x;

		float v1x = x2-x1;
		float v1y = y2-y1;
		float v1z = z2-z1;

		float v2x = x3-x1;
		float v2y = y3-y1;
		float v2z = z3-z1;

		float nvx = v1y*v2z - v1z*v2y;
		float nvy = v1z*v2x - v1x*v2z;
		float nvz = v1x*v2y - v1y*v2x;

		float a = nvx;
		float b = nvy;
		float c = nvz;
		float d = -(a*x1 + b*x2 + c*x3);

		for (int index=0; index < cloud->points.size(); index++){

			if(inliersCandidate.count(index))
				continue;

			float px = cloud->points[index].x;
			float py = cloud->points[index].y;
			float pz = cloud->points[index].z;

			float dist = (a*px + b*py +  c*pz - d)/sqrt(a*a+b*b+c*c);

			if (fabs(dist) < distanceTol){
				inliersCandidate.insert(index);
				//std::cout << "sist:" << dist << "\t thres " << distanceTol << std::endl;
				//std::cout << "abs: " << abs(dist) << "\t thres " << distanceTol << std::endl;
				//std::cout << "fabs:" << fabs(dist) << "\t thres " << distanceTol << std::endl;
				//std::cout << std::endl;
			}

		if(inliersCandidate.size() > inliersResult.size())
			inliersResult = inliersCandidate;
		}
	}

return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // Add several random points to vizualize and debug plane equation

	/*for (int j=0; j<500; j++){
		double rx = 40*(((double) rand() / (RAND_MAX))-0.5);
		double ry = 40*(((double) rand() / (RAND_MAX))-0.5);
		double rz = 40*(((double) rand() / (RAND_MAX))-0.5);

		pcl::PointXYZ point;
		point.x = rx;
		point.y = ry;
		point.z = rz;

		cloud->points.push_back(point);
	}*/

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 500, 0.4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}

}
