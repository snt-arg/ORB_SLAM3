#pragma once

#include <iostream>
#include <vector>

namespace ORB_SLAM3
{

class ToA {

  
public:
  // Input sensor
  enum eStatus { NoMeasAdded, AddedMeas };

private:
  eStatus status_;
  double TimeStamp_;
  std::vector<double> ToaMeas_;
  std::vector<std::vector<double>> BsPositions_ = {{10, 15, 13}, {14, 17, 18}, {-10, 20, 7}};

public:
  // Constructor
  //ToA();
  ToA(const double TimeStamp, const std::vector<double> toaMeas , const eStatus status = AddedMeas);

  //Set params
  void SetTimeStamp(double TimeStamp);
  void SetBsPositions(std::vector<std::vector<double>> BsPositions);
  void SetToAMeas(std::vector<double> toaMeas);

  //Get Params
  double GetTimeStamp();
  std::vector<double> GetToaMeas();
  eStatus GetStatus();
  std::vector<std::vector<double>> GetBsPositions();
};

}