/**
 * @Author: Yusu Pan <yuthon>
 * @Date:   2017-06-05T12:26:40+08:00
 * @Email:  xxdsox@gmail.com
 * @Project: humanoid
 * @Filename: idetector.hpp
 * @Last modified by:   yuthon
 * @Last modified time: 2017-06-05T12:27:16+08:00
 * @Copyright: ZJUDancer
 */

#pragma once
#include "dvision/parameters.hpp"
#include "dvision/utils.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/opencv.hpp>
namespace dvision {
class IDetector
{
  public:
    virtual bool Init() = 0;
    virtual ~IDetector(){};
};

} // namespace dvision
