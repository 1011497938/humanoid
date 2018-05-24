#pragma once
#include <opencv2/opencv.hpp>
#include <dirent.h>
#include <string>
#include <vector>
#include "dvision/frame.hpp"


namespace dvision{
class CameraDummy
{
public:
  CameraDummy(std::string imageFolder)
  : imagesNum(0),
    imageCnt(0)
  {
      init(imageFolder);
  }
  bool init(std::string imageFolder);
  Frame capture();

private:
  std::vector<std::string> imagesName;
  int imagesNum;
  int imageCnt;
  std::string path;


};
}
