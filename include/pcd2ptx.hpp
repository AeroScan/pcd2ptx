#pragma once
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class Pcd2PtxConverter
{
    private:

    std::string inputFile;
    std::string outputFile;
    void writePTXHeader(std::ofstream &file, const int width, const int height);
    void writePTXFileXYZ(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void writePTXFileXYZRGB(const std::string &filename, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    public:
    bool execute();
    void setInputFile(std::string file);
    void setOutputFile(std::string file);

};