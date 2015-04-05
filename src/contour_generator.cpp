#include <contour_generator/contour_generator.h>

ContourGenerator::ContourGenerator() : viewer_(new pcl::visualization::PCLVisualizer ("Contour Viewer"))
{
}

ContourGenerator::~ContourGenerator()
{
}

PointCloudPtr ContourGenerator::getContour(const std::vector<CoordinateT > &coordinates)
{
    PointCloudPtr contour(new PointCloud);
    for (int i = 0; i < coordinates.size(); i++)
    {
        PointT start_point;
        start_point.x = coordinates.at(i).at(0);
        start_point.y = coordinates.at(i).at(1);
        start_point.z = 0.0;

        PointT end_point;
        if (i < coordinates.size() - 1)
        {
            end_point.x = coordinates.at(i + 1).at(0);
            end_point.y = coordinates.at(i + 1).at(1);
            end_point.z = 0.0;
        }
        else
        {
            end_point.x = coordinates.at(0).at(0);
            end_point.y = coordinates.at(0).at(1);
            end_point.z = 0.0;
        }

        int num_steps_per_segment = 40;
        double x_step_size = (end_point.x - start_point.x) / num_steps_per_segment;
        double y_step_size = (end_point.y - start_point.y) / num_steps_per_segment;

        for (int j = 0; j < num_steps_per_segment; j++)
        {
            PointT point;
            point.x = start_point.x + (j * x_step_size);
            point.y = start_point.y + (j * y_step_size);
            point.z = 0.0;
            contour->points.push_back(point);
        }
    }
    contour->width = contour->points.size();
    contour->height = 1;
    return contour;
}
std::vector<CoordinateT> ContourGenerator::getRectanglePoints(double length, double width)
{
    std::vector<CoordinateT> points;
    CoordinateT top_right {length / 2.0, width / 2.0};
    CoordinateT bottom_right {length / 2.0, -width / 2.0};
    CoordinateT bottom_left {-length / 2.0, -width / 2.0};
    CoordinateT top_left {-length / 2.0, width / 2.0};
    points.push_back(top_right);
    points.push_back(bottom_right);
    points.push_back(bottom_left);
    points.push_back(top_left);
    return points;
}

std::vector<CoordinateT> ContourGenerator::getHexagonPoints(double side_length)
{
    std::vector<CoordinateT> points;
    double in_radius = std::sqrt(std::pow(side_length, 2.0) - std::pow(side_length / 2.0, 2.0));

    CoordinateT top {0.0, side_length};
    CoordinateT top_right {in_radius, side_length / 2.0};
    CoordinateT bottom_right {in_radius, -side_length / 2.0};
    CoordinateT bottom {0.0, -side_length};
    CoordinateT bottom_left {-in_radius, -side_length / 2.0};
    CoordinateT top_left {-in_radius, side_length / 2.0};

    points.push_back(top);
    points.push_back(top_right);
    points.push_back(bottom_right);
    points.push_back(bottom);
    points.push_back(bottom_left);
    points.push_back(top_left);
    return points;
}

std::vector<CoordinateT> ContourGenerator::getBoltPoints(double head_diameter, double head_thickness,
                                                         double nominal_length, double major_diameter)
{
    std::vector<CoordinateT> points;
    double full_length = nominal_length + head_thickness;

    CoordinateT top_right {full_length / 2.0, head_diameter / 2.0};
    CoordinateT bottom_right {full_length / 2.0, -head_diameter / 2.0};
    CoordinateT bottom_middle {(full_length / 2.0) - head_thickness, -head_diameter / 2.0};
    CoordinateT bottom_m_middle {(full_length / 2.0) - head_thickness, -major_diameter / 2.0};
    CoordinateT bottom_left {-full_length / 2.0, -major_diameter / 2.0};
    CoordinateT top_left {-full_length / 2.0, major_diameter / 2.0};
    CoordinateT top_m_middle {(full_length / 2.0) - head_thickness, major_diameter / 2.0};
    CoordinateT top_middle {(full_length / 2.0) - head_thickness, head_diameter / 2.0};

    points.push_back(top_right);
    points.push_back(bottom_right);
    points.push_back(bottom_middle);
    points.push_back(bottom_m_middle);
    points.push_back(bottom_left);
    points.push_back(top_left);
    points.push_back(top_m_middle);
    points.push_back(top_middle);
    return points;
}

PointCloudPtr ContourGenerator::S40_40_H()
{
    return getContour(getRectanglePoints(0.1, 0.04));
}

PointCloudPtr ContourGenerator::F20_20_H()
{
    return getContour(getRectanglePoints(0.1, 0.02));
}

PointCloudPtr ContourGenerator::M20_100_H()
{
    return getContour(getBoltPoints(0.04, 0.019, 0.1, 0.028));
}

PointCloudPtr ContourGenerator::M20_H()
{
    return getContour(getHexagonPoints(0.0195));
}

PointCloudPtr ContourGenerator::M30_H()
{
    return getContour(getHexagonPoints(0.02875));
}

PointCloudPtr ContourGenerator::R20_H()
{
    return getContour(getRectanglePoints(0.04875, 0.034));
}

void ContourGenerator::view(const PointCloudPtr &cloud)
{
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addPointCloud<PointT>(cloud, "contour");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "contour");

    viewer_->addCoordinateSystem(0.1, 0);
    viewer_->initCameraParameters();
    viewer_->setCameraPosition(0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0);
    while (!viewer_->wasStopped())
    {
        viewer_->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}
