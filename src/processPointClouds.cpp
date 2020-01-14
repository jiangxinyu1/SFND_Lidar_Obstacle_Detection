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

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template< typename PointT>
std::pair< typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr > ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // 创建两个点云，障碍物点云和平面点云
    typename pcl::PointCloud<PointT>::Ptr planeCloud( new pcl::PointCloud<PointT>() );
    typename pcl::PointCloud<PointT>::Ptr obstCloud( new pcl::PointCloud<PointT>() );
    for(  int index : inliers->indices )
    {
        planeCloud->points.push_back( cloud->points[index]);
    }
    // 创建点云提取对象
    pcl::ExtractIndices<PointT> extract;

    // 设置输入点云  
    extract.setInputCloud (cloud);

    // 设置分割后的内点为需要提取的点集
    extract.setIndices(inliers);

    // 设置提取外点而非内点
    extract.setNegative(true);

    // 把滤波之后的点云存储在obstCloud里
    extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult( obstCloud, planeCloud );
    return segResult;
}


/*
    SegmentPlane函数---> 输入点云，把障碍物和地面的点云分割
*/
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // 创建存储内点的点索引集合对象inliers
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // 创建分割时所需要的模型系数对象
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    // 创建分割对象seg
    pcl::SACSegmentation <PointT> seg;

    // 设置模型系数需要优化（可选择配置）
    seg.setOptimizeCoefficients (true);

    // 设置分割的模型类型
    seg.setModelType (pcl::SACMODEL_PLANE);

    // 设置最大迭代次数
    seg.setMaxIterations (maxIterations); 

    // 设置采用的随机参数估计方法
    seg.setMethodType (pcl::SAC_RANSAC);

    // 设置距离阈值
    seg.setDistanceThreshold (distanceThreshold);

    // 设置输入点云
    seg.setInputCloud(cloud);

    // 引发分割实现，存储分割结果到点集合inliers
    // 存储平面模型的系数coefficients
    seg.segment ( *inliers , *coefficients);

    if( inliers->indices.size()==0)
    {
        std::cout << "Could not estimate a planner model !!! " << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    // 调用 SeparateClouds(inliers,cloud); 函数，把分割的结果存储到 segResult(pair类型)里面
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

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