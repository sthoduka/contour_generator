#include <contour_generator/contour_generator.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << "<name of contour> " << "[options]" << std::endl;
        std::cout << "\toptions:" << std::endl;
        std::cout << "\t\t-s Saves pointcloud to <name of contour>.pcd" << std::endl;
        return 1;
    }

    ContourGenerator cg;
    PointCloudPtr pointcloud;
    if (strcmp(argv[1], "S40_40_H") == 0)
    {
        pointcloud = cg.S40_40_H();
    }
    else if (strcmp(argv[1], "F20_20_H") == 0)
    {
        pointcloud = cg.F20_20_H();
    }
    else if (strcmp(argv[1], "M20_100_H") == 0)
    {
        pointcloud = cg.M20_100_H();
    }
    else if (strcmp(argv[1], "M20_H") == 0)
    {
        pointcloud = cg.M20_H();
    }
    else if (strcmp(argv[1], "M30_H") == 0)
    {
        pointcloud = cg.M30_H();
    }
    else if (strcmp(argv[1], "R20_H") == 0)
    {
        pointcloud = cg.R20_H();
    }
    else
    {
        std::cout << "Contour for " << argv[1] << " not defined" << std::endl;
        return 1;
    }
    if (argc > 2)
    {
        for (int i = 0; i < argc - 2; i++)
        {
            char * option = argv[i + 2];
            if (strcmp(option, "-s") == 0)
            {
                std::string filename(argv[1]);
                filename.append(".pcd");
                pcl::io::savePCDFileASCII(filename, *pointcloud);
                std::cout << "Saved file: " << filename << std::endl;
            }
        }
    }
    cg.view(pointcloud);
    return 0;
}
