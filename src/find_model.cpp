//project
#include "model_building.cpp"

//PCL
//segmentation
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

void findPlane(ColorACloudPtr cloud, pcl::PointIndices::Ptr inliers){
    //perform plane segmentation
    pcl::ModelCoefficients coefficients;
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud->makeShared ());
    seg.segment (*inliers, coefficients);
}
void extractIndices(ColorACloudPtr cloud, pcl::PointIndices::Ptr indices, bool negative = true){
    ColorACloud tmp;

    pcl::ExtractIndices<ColorAPoint> eifilter;
    eifilter.setInputCloud (cloud);
    eifilter.setIndices (indices);
    eifilter.setNegative (negative);
    eifilter.filter (tmp);

    *cloud = tmp;
}
void findClusters(ColorACloudPtr cloud, vector<pcl::PointIndices> *cluster_indices){
    pcl::search::KdTree<ColorAPoint>::Ptr tree (new pcl::search::KdTree<ColorAPoint>);
    tree->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<ColorAPoint> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (*cluster_indices);
}
void rotate1(Eigen::Matrix4f *transform, float theta){
    Eigen::Matrix4f tmp;
    tmp <<  cos(theta),     -sin(theta),    0,  0,
            sin(theta),     cos(theta),     0,  0,
            0,              0,              1,  0,
            0,              0,              0,  1;
    *transform *= tmp;
}
void rotate2(Eigen::Matrix4f *transform, float theta){
    Eigen::Matrix4f tmp;
    tmp <<  1,      0,          0,          0,
            0,      cos(theta), -sin(theta),0,
            0,      sin(theta), cos(theta), 0,
            0,      0,          0,          1;
    *transform *= tmp;
}
void rotateCloud(ColorACloudPtr cloud_in, ColorACloudPtr cloud_out, float theta, float alpha){
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    rotate1(&transform,theta);
    rotate2(&transform,alpha);
    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);
}
void centerCloud(ColorACloudPtr cloud){
    ColorACloud tmp;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*cloud, centroid);

    Eigen::Matrix4f transform;
    transform <<    1,0,0,-centroid.x(),
                    0,1,0,-centroid.y(),
                    0,0,1,-centroid.z(),
                    0,0,0,1;
    pcl::transformPointCloud(*cloud, tmp, transform);
    *cloud = tmp;
}
int main(int argc, char *argv[]){
    ColorACloudPtr cloud (new ColorACloud);

    if (pcl::io::loadPCDFile("test/frame-1.pcd", *cloud) == -1)
        return -1;

    //find the table plane
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    findPlane(cloud,inliers);
    //remove it
    extractIndices(cloud,inliers);

    //find the objects
    std::vector<pcl::PointIndices> cluster_indices;
    findClusters(cloud,&cluster_indices);
    //take the first one
    pcl::PointIndices::Ptr cluster (new pcl::PointIndices(cluster_indices[0]));
    extractIndices(cloud,cluster,false);

    //rotate the cloud into many different orientations and save to disk
    int index = 0;
    for(float i = 0; i<2*M_PI; i+=1){
        for(float j = 0; j<M_PI; j+=1){
            ColorACloudPtr tmp (new ColorACloud);
            rotateCloud(cloud,tmp,i,j);
            centerCloud(tmp);
            pcl::io::savePCDFileASCII ((boost::format("output_rotated_%d.pcd") % index).str(), *tmp);
            index ++;
        }
    }
}
