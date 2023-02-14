#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

void LoadTOA(const string strToaPath, vector<double> &vTimeStamps,  vector<vector<double>> &vToa,  int Bs_num) ;


int main(int argc, char *argv[])
{

    if(argc < 5)
    {
        cerr << endl << "Usage: ... " << endl;
        return 1;
    }

 const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector< vector< vector< double> > > vToa;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);
    vToa.resize(num_seq);

    int tot_images = 0;

     for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq +"/mav0/cam0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";
        string pathToa = pathSeq + "/mav0/toa/data.csv";

        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        // TODO: Bs_num must be read from config file
        cout << "Loading toa for sequence " << seq << "...";
        LoadTOA(pathToa, vTimestampsImu[seq], vToa[seq], 3);     
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }



    //debuggin printing the toa read value
  // cout << "this si the size of the myvector" << vToa.size() << endl;
  //cout << sizeof(vToa);
  //for (int i = 0; i < vToa[0].size(); i++) {
  //  for (int j = 0; j < vToa[0][i].size(); j++) {
  //    cout << vToa[0][i][j] << " ";
  //  }
  //}



}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(5000);
    vAcc.reserve(5000);
    vGyro.reserve(5000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}



void LoadTOA(const string strToaPath, vector<double> &vTimeStamps,
             vector<vector<double>> &vToa, int Bs_num) {
  ifstream fToa;
  fToa.open(strToaPath);
  while (!fToa.eof()) {
    string s;
    size_t pos = 0;
    string item;
    int count = 0;
    vector<double> data(Bs_num+1);
    // The first is time_stamp, others are TOAs (or estimated distance)
    if (fToa.is_open()) {
      getline(fToa, s);
    } else {
      cout << "Error: Unable to open file." << endl;
      break;
    }
    if (s==""){break;}
    while ((pos = s.find(',')) != string::npos) {
      item = s.substr(0, pos);
      data[count++] = stod(item);
      s.erase(0, pos + 1);
    }
    // copy(data.begin(), data.end(), ostream_iterator<int>(cout, " "));
    item = s.substr(0, pos);
    data[Bs_num] = stod(item);
    vTimeStamps.push_back(data[0]);
    vToa.push_back(data);
  }
}