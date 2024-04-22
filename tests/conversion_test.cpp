#include <iostream>
#include <pcd2ptx.hpp>

int main(int argc, char **argv)
{
    Pcd2PtxConverter converter;
    converter.setInputFile("pca-modelo.pcd");
    converter.setOutputFile("pca-modelo.ptx");
    converter.execute();
}