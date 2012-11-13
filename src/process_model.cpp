#include <algorithm>

#include "model_building.cpp"

#include <pcl/surface/convex_hull.h>
#include <pcl/ros/conversions.h>
#include <pcl/PolygonMesh.h>
#include "pcl/io/pcd_io.h" 
#include "pcl/io/vtk_io.h" 

using namespace std;

int main(int argc, char *argv[]){
    if(argc>1){
        CloudPtr cloud (new Cloud);
        ColorCloudPtr color_cloud (new ColorCloud);
        if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file.\n");
            return -1;
        }
        for(int i = 0; i<cloud->size(); i++){
            Point p1 = cloud->points[i];
            ColorPoint p2;
            p2.x = p1.x;
            p2.y = p1.y;
            p2.z = p1.z;
            p2.r = 0;
            p2.g = 0;
            p2.b = 0;
            color_cloud->push_back(p2);
        }
        
        float centroid[3];
        calculateCentroid(cloud,centroid);


        pcl::PolygonMesh pm;
        pcl::ConvexHull<ColorPoint> chull;
        chull.setInputCloud(color_cloud);
        chull.reconstruct(pm);

        pcl::fromROSMsg(pm.cloud,*color_cloud);
        vector<float> distances(color_cloud->size(),999999);

        for(int i = 0; i<pm.polygons.size(); i++){
            int i0 = pm.polygons[i].vertices[0];
            int i1 = pm.polygons[i].vertices[1];
            int i2 = pm.polygons[i].vertices[2];
            ColorPoint p0 = color_cloud->points[i0];
            ColorPoint p1 = color_cloud->points[i1];
            ColorPoint p2 = color_cloud->points[i2];

            Eigen::Vector3f v0;
            Eigen::Vector3f v1;
            Eigen::Vector3f v2;
            v0 << p0.x,p0.y,p0.z;
            v1 << p1.x,p1.y,p1.z;
            v2 << p2.x,p2.y,p2.z;
            Eigen::Vector3f normal = (v1-v0).cross(v2-v0).normalized();

            Eigen::Vector3f p;
            p << centroid[0], centroid[1], centroid[2];

            Eigen::Vector3f to_p = p-v0;
            Eigen::Vector3f projected_p = p-(normal* normal.dot(to_p));

            float dist0 = (v1-v0).dot(projected_p-v0)/(v1-v0).norm();
            float dist1 = (v2-v0).dot(projected_p-v0)/(v2-v0).norm();
            distances[i0] = min(distances[i0],(dist0+dist1));
            distances[i1] = min(distances[i1],(dist0+dist1));
            distances[i2] = min(distances[i2],(dist0+dist1));
        }
        float max_dist = *max_element(distances.begin(),distances.end());

        float thresh = 500;
        for(int i = 0; i<color_cloud->size(); i++){
            ColorPoint* p1 = &color_cloud->points[i];
            float color = 255 - distances[i]*(255/max_dist);
            if(distances[i]>thresh)
                color = 0;
            p1->r = color;
            p1->g = 0;
            p1->b = 0;
        }

        chull.setInputCloud(color_cloud);
        chull.reconstruct(pm);

        pcl::io::saveVTKFile("hull.vtk",pm);
    }
}
