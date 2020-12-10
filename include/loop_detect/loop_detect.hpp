#ifndef LOOP_DETECT_H
#define LOOP_DETECT_H

#include <iostream>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// TODO: 匹配算法抽象类   熟悉了多种回环的方法后, 将代码架构改成 基类 - 子类的模板模式

class Loopdetect
{
public:
    Loopdetect()
    {}
    
    virtual ~Loopdetect()  
    {}
    
    // User-side API
    template<typename T>
    void ExtractInfomationFromScanForLoopDetect( pcl::PointCloud<T> & _scan_down );

    std::pair<int, float> DetectLoopClosureID( void );

protected:
    

}; //class Loopdetect

#endif