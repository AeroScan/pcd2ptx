#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcd2ptx.hpp>

void Pcd2PtxConverter::writePTXHeader(std::ofstream &file, const int width, const int height)
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

void Pcd2PtxConverter::writePTXFileXYZ(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::ofstream file(filename.c_str());

    writePTXHeader(file, cloud->width, cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        file << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << " " << 1 << "\n";
    }

    file.close();
}

void Pcd2PtxConverter::writePTXFileXYZRGB(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
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

bool Pcd2PtxConverter::execute()
{
    // Load the point cloud data
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(inputFile, *cloud);

    // Save to PTX format
    writePTXFileXYZRGB(outputFile, cloud);

    return true;
}

void Pcd2PtxConverter::setInputFile(std::string file)
{
    inputFile = file;
}

void Pcd2PtxConverter::setOutputFile(std::string file)
{
    outputFile = file;
}