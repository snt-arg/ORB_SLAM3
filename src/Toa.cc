#include <Toa.h>

namespace ORB_SLAM3
{

ToA::ToA(const double TimeStamp , const double toaMeas ,const std::vector<double> BsPosition , const int BsId  , const ToA::eStatus status):
  ToaValue_(toaMeas), BsPosition_(BsPosition), TimeStamp_(TimeStamp), BsId_(BsId),status_(status)
{
  
}

void ToA::SetBsPosition(std::vector<double> BsPosition) {
  BsPosition_ = BsPosition;
}
double ToA::GetTimeStamp(){return TimeStamp_;}
double ToA::GetToa() { return ToaValue_; }
int ToA::GetBsId () {return BsId_;}
std::vector<double> ToA::GetBsPosition() { return BsPosition_; }
void ToA::SetToA(double toaMeas) {ToaValue_ = toaMeas; }
void ToA::SetTimeStamp(double TimeStamp) {TimeStamp_ = TimeStamp;}
void ToA::SetBsId (int BsId) {BsId_ = BsId;}


}