#pragma once

#include <iostream>
#include <vector>

namespace ORB_SLAM3
{

class ToA {

  // Constructor
public:
  // Input sensor
  enum eStatus {
    NoMeas = 0,
    AddedMeas = 1,
  };

private:
  eStatus status_;
  double TimeStamp_;
  double ToaValue_;
  int BsId_;
  std::vector<double> BsPosition_;

public:
  ToA(const double TimeStamp = 0, const double toaMeas = 0,
      const std::vector<double> BsPosition = std::vector<double>(),
      const int BsId = 0, const eStatus status = AddedMeas);
  double GetTimeStamp();
  double GetToa();
  eStatus GetStatus();
  int GetBsId();
  std::vector<double> GetBsPosition();
  void SetToA(double toaMeas);
  void SetBsPosition(std::vector<double> BsPosition);
  void SetTimeStamp(double TimeStamp);
  void SetBsId(int BsId);
};

}