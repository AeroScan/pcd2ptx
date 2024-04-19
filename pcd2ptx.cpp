#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

void writePTXHeader(std::ofstream &file, const int width, const int height)
{
    file << width << std::endl
         << height << std::endl;

    file << 0.f << " " << 0.f << " " << 0.f << std::endl;
    file << 1.f << " " << 0.f << " " << 0.f << std::endl;
    file << 0.f << " " << 1.f << " " << 0.f << std::endl;
    file << 0.f << " " << 0.f << " " << 1.f << std::endl;

    file << 1.f << " " << 0.f << " " << 0.f << " " << 0.f << std::endl;
    file << 0.f << " " << 1.f << " " << 0.f << " " << 0.f << std::endl;
    file << 0.f << " " << 0.f << " " << 1.f << " " << 0.f << std::endl;
    file << 0.f << " " << 0.f << " " << 0.f << " " << 1.f << std::endl;
}

void writePTXFileXYZ(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::ofstream file(filename.c_str());

    writePTXHeader(file, cloud->width, cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << 1 << "\n";
    }

    file.close();
}

void writePTXFileXYZRGB(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    std::ofstream file(filename.c_str());

    writePTXHeader(file, cloud->width, cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        std::uint32_t rgb = *reinterpret_cast<int *>(&(cloud->points[i].rgb));
        std::uint8_t r = (rgb >> 16) & 0x0000ff;
        std::uint8_t g = (rgb >> 8) & 0x0000ff;
        std::uint8_t b = (rgb) & 0x0000ff;
        file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << 1 << " " << (int)r << " " << (int)g << " " << (int)b << "\n";
    }

    file.close();
}

int main()
{
    // Load the point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile("pc.pcd", *cloud);

    // Save to PTX format
    writePTXFileXYZRGB("pc.ptx", cloud);

    return 0;
}