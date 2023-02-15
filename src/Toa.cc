#include <Toa.h>

namespace ORB_SLAM3
{

  //ToA::ToA() : status_(NoMeasAdded) {}
  ToA::ToA(const double TimeStamp, const std::vector<double> toaMeas , const ToA::eStatus status):
  TimeStamp_(TimeStamp), ToaMeas_(toaMeas), status_(status)
{
  
}

//Set params 
void ToA::SetTimeStamp(double TimeStamp) {TimeStamp_ = TimeStamp;}
void ToA::SetToAMeas(std::vector<double> toaMeas) {ToaMeas_ = toaMeas; }
void ToA::SetBsPositions(std::vector<std::vector<double>> BsPositions) { BsPositions_ = BsPositions;}

//Get Params
double ToA::GetTimeStamp(){return TimeStamp_;}
ToA::eStatus ToA::GetStatus(){return status_;}
std::vector<double> ToA::GetToaMeas() { return ToaMeas_; }
std::vector<std::vector<double>> ToA::GetBsPositions() { return BsPositions_; }

}