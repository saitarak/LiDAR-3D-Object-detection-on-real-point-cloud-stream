/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>
#include<stdio.h>
#include <complex>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
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

//Ransac 2D
std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
        srand(time(NULL));
	// For max iterations 
	    int maxinliers = 0;
        for (int i=0; i<maxIterations; i++)
        {
			std::unordered_set<int> inliers;
            int index1 = rand()%(cloud->points.size());
			int index2 = rand()%(cloud->points.size());
			pcl::PointXYZ point;
            float x1 = cloud->points[index1].x;
			float y1 = cloud->points[index1].y; 
            float x2 = cloud->points[index2].x;
			float y2 = cloud->points[index2].y;
			float m = (y2-y1)/(x2-x1);
			float a = y1-y2;
			float b = x2-x1; 
			float c = x1*y2 - x2*y1;
			float d = 0;
			int count = 0;
			for (int index=0;index<cloud->points.size();index++)
			{
				if(inliers.count(index)>0)
		            continue;
				float d = fabs(a*cloud->points[index].x+b*cloud->points[index].y+c)/sqrt(a*a+b*b);
				if (d<=distanceTol)
				{
					count = count + 1;  
					inliers.insert(index);
				}	
			}
            if (inliers.size()>inliersResult.size())
			{
				inliersResult = inliers;
			}
			if (count>maxinliers)
			{
				maxinliers = count;
			}
        }
		std::cout << maxinliers;
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

//Ransac 3D
std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
        srand(time(NULL));
	// For max iterations 
	    int maxinliers = 0;
        for (int n=0; n<maxIterations; n++)
        {
			typedef std::complex<double> dcomp;
			std::unordered_set<int> inliers;
            int index1 = rand()%(cloud->points.size());
			int index2 = rand()%(cloud->points.size());
			int index3 = rand()%(cloud->points.size());
			pcl::PointXYZ point;
            float x1 = cloud->points[index1].x;
			float y1 = cloud->points[index1].y;
			float z1 = cloud->points[index1].z;
            float x2 = cloud->points[index2].x;
			float y2 = cloud->points[index2].y;
			float z2 = cloud->points[index2].z;
            float x3 = cloud->points[index3].x;
			float y3 = cloud->points[index3].y;
			float z3 = cloud->points[index3].z;
			float a = ((y2-y1)*(z3-z1)-(z2-z1)*(y3-y2));
			float b = ((z2-z1)*(x3-x1)-(x2-x1)*(z3-z2));
			float c = ((x3-x1)*(y3-y1)-(y2-y1)*(x3-x2));
			float d = -(a*x1+b*y1+c*z1);
			int count = 0;
			for (int index=0;index<cloud->points.size();index++)
			{
				if(inliers.count(index)>0)
		            continue;
				float d1 = fabs(a*cloud->points[index].x+b*cloud->points[index].y+c*cloud->points[index].z+d)/sqrt(a*a+b*b+c*c);
				if (d1<=distanceTol)
				{
					count = count + 1;  
					inliers.insert(index);
				}	
			}
            if (inliers.size()>inliersResult.size())
			{
				inliersResult = inliers;
			}
			if (count>maxinliers)
			{
				maxinliers = count;
			}
        }
		std::cout << maxinliers;
	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 1.0);
	std::unordered_set<int> inliers = RansacPlane(cloud, 30, 1.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
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
