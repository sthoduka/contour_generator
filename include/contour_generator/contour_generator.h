#ifndef CONTOUR_GENERATOR_H_
#define CONTOUR_GENERATOR_H_

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef std::vector<double> CoordinateT;

class ContourGenerator
{
    public:
        ContourGenerator();
        virtual ~ContourGenerator();

        /**
         * Returns 3D contours for cavities of objects when placed horizontally
         * Cavities can be seen in Figure 6.3 on page 34 here:
         * http://robocupatwork.org/download/rulebook-2014-07-06.pdf
         */
        PointCloudPtr S40_40_H();
        PointCloudPtr F20_20_H();
        PointCloudPtr M20_100_H();
        PointCloudPtr M20_H();
        PointCloudPtr M30_H();
        PointCloudPtr R20_H();

        void view(const PointCloudPtr &cloud);

    private:
        /**
         * Generates 3D points (on the x-y plane) using coordinates as waypoints
         * Closes the contour by adding points between the first and last coordinate
         */
        PointCloudPtr getContour(const std::vector<CoordinateT> &coordinates);

        /**
         * Returns coordinates of the 4 corners of a rectangle
         */
        std::vector<CoordinateT> getRectanglePoints(double length, double width);

        /**
         * Returns coordinates of the 6 vertices of a regular hexagon
         */
        std::vector<CoordinateT> getHexagonPoints(double side_length);

        /**
         * Returns coordinates for the corners of an outline of a bolt
         */
        std::vector<CoordinateT> getBoltPoints(double head_diameter, double head_thickness,
                                               double nominal_length, double major_diameter);

    private:
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

};

#endif
