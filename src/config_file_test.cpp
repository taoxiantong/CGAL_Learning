#include "config_file.h"
#include <string>


int main(int argc, char * argv[])
{
    std::string file("1.cfg");
    ConfigFile config_file(file);
    config_file.ExtractKeys();
    
    // 读取一个值，并给出默认值
    // cfg中没有该关键字，查找不到，赋给默认值
    bool Vbool = config_file.getValueOfKey<bool>("test1", false);

    // 查找关键字返回int类型的vector
    std::vector<int> Vvint = config_file.getValueOfKeyAsStdVectorInt("test2", "2");

    // 查找关键字返回double类型
    double Vd = config_file.getValueOfKey<double>("test3", 20.0);

    // 查找关键字返回int类型
    int Vint =  config_file.getValueOfKey<int>("test4", 6);

    // 查找关键字返回string类型
    std::string Vstring = config_file.getValueOfKeyAsString("test5", "");

    // 查找关键字返回string类型的 vector
    std::vector<double> Vvstring = config_file.getValueOfKeyAsStdVectorDouble("test6", "-1 1 -1 1 -1 1");
}