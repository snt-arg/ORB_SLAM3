#include <Toa.h>

namespace ORB_SLAM3
{

  ToA::ToA() : status_(NoMeasAdded) {}
  ToA::ToA(const double TimeStamp, const std::vector<double> toaMeas , const ToA::eStatus status):
  TimeStamp_(TimeStamp), ToaMeas_(toaMeas), status_(status)
{
  
}

//Set params 
// std::vector<std::vector<double>> ORB_SLAM3::ToA::sBsPositions = {{5, 6, 3}, {-6, 15, 5}, {10, -3, 8}};
std::vector<std::vector<double>> ORB_SLAM3::ToA::sBsPositions {{-0.33417888,  5.30332756,  9.21750544}, {-1.58969631, -7.41698016, 15.74571016}, {7.55860966, 11.76619995,  4.02761977}};
void ToA::SetTimeStamp(double TimeStamp) {TimeStamp_ = TimeStamp;}
void ToA::SetToAMeas(std::vector<double> toaMeas) {ToaMeas_ = toaMeas; }
// void ToA::SetBsPositions(std::vector<std::vector<double>> BsPositions) { sBsPositions = BsPositions;}

//Get Params
double ToA::GetTimeStamp(){return TimeStamp_;}
ToA::eStatus ToA::GetStatus(){return status_;}
std::vector<double> ToA::GetToaMeas() { return ToaMeas_; }
// std::vector<std::vector<double>> ToA::GetBsPositions() { return BsPositions_; }

}
