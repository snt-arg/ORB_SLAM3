//This is developed for now just to work with one sequence, 
//for more sequences just it is required to take care of TOA dataset, the waw they are read
//no need to other changes

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>

#include<opencv2/core/core.hpp>

#include<System.h>
#include<Toa.h>
#include "ImuTypes.h"

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

void LoadTOA(const string strToaPath, vector<double> &vTimeStamps,  vector<vector<double>> &vToa,  int Bs_num) ;

// std::vector<std::vector<double>> ORB_SLAM3::ToA::sBsPositions = {{6,	7, 5}, {-26,-70, 10}, {-120,	40,	15}};
std::vector<std::vector<double>> ORB_SLAM3::ToA::sBsPositions = {{5, 6, 3}, {-6, 15, 5}, {10, -3, 8}};


double ttrack_tot = 0;
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

    // const vector<vector<double>> bs_positions = {{1, 20, 3}, {4, 50, 6}, {7, 80, 9}};
     ORB_SLAM3::ToA::sBsPositions = {{5, 6, 3}, {-6, 15, 5}, {10, -3, 8}};

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageFilenames;
    vector< vector<double> > vTimestampsCam;
    vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    vector< vector< vector< double> > > vToaAll;
    vector< vector<double> > vTimestampsToa;
    vector<int> nImages;
    vector<int> nImu;
    vector<int> first_imu(num_seq,0);
    vector<int> toaMeas_ind(num_seq,0);

    vstrImageFilenames.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    vAcc.resize(num_seq);
    vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    nImages.resize(num_seq);
    nImu.resize(num_seq);
    vToaAll.resize(num_seq);
    vTimestampsToa.resize(num_seq);

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
        LoadTOA(pathToa, vTimestampsToa[seq], vToaAll[seq], 3);     
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

    //for Debugging:
        //ORB_SLAM3::ToA obj(vTimestampsToa[0][0], vToa[0][0]);
        //cout<<"This is the timestamp read  --- "<<obj.GetTimeStamp()<<endl;
        //cout<<"this is another message  -- "<<obj.GetStatus()<<endl;


      // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR, true); 
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true); 
    //TODO the sensor must be changed to IMU_MONOCULAR_TOA
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    int proccIm=0;  



     for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        for(int ni=0; ni<nImages[seq]; ni++, proccIm++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //CV_LOAD_IMAGE_UNCHANGED);

            double tframe = vTimestampsCam[seq][ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  vstrImageFilenames[seq][ni] << endl;
                return 1;
            }

            if(imageScale != 1.f)
            {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
                int width = im.cols * imageScale;
                int height = im.rows * imageScale;
                cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
                std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
                std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
                t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
                SLAM.InsertResizeTime(t_resize);
#endif
            }

            // Load imu measurements from previous frame
            vImuMeas.clear();

            if(ni>0)
            {
                // cout << "t_cam " << tframe << endl;

                while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])//tframe
                {
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAcc[seq][first_imu[seq]].x,vAcc[seq][first_imu[seq]].y,vAcc[seq][first_imu[seq]].z,
                                                             vGyro[seq][first_imu[seq]].x,vGyro[seq][first_imu[seq]].y,vGyro[seq][first_imu[seq]].z,
                                                             vTimestampsImu[seq][first_imu[seq]]));
                    first_imu[seq]++;
                }
            }

                        if(ni>0)
            {
                while(vTimestampsToa[seq][toaMeas_ind[seq]+1]<=tframe )
                {
                    toaMeas_ind[seq]++;
                }
                if (vTimestampsToa[seq][toaMeas_ind[seq]+1] - tframe < tframe - vTimestampsToa[seq][toaMeas_ind[seq]]) {toaMeas_ind[seq]++;}
            }


    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the image to the SLAM system
            // cout << "tframe = " << tframe << endl;
             if (abs(tframe - vTimestampsToa[seq][toaMeas_ind[seq]]) < 0.1 ){
            SLAM.TrackMonocularToa(im,tframe, vToaAll[seq][toaMeas_ind[seq]], vImuMeas); // TODO change to monocular_inertial
            }else{
                 SLAM.TrackMonocularToa(im,tframe, vector<double>(1,0), vImuMeas);
            }

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            ttrack_tot += ttrack;
            // std::cout << "ttrack: " << ttrack << std::endl;

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }
        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string abs_path = "/home/meisam/ORB_SLAM3/Results/";
        const string kf_file =  abs_path+"kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  abs_path+"f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;





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
    //TODO-msm the time stamp in euroc is in 1e+18 but here they are using 1e-9 (Euroc time stamp is in nanosec here is sec)
    item = s.substr(0, pos);
    data[Bs_num] = stod(item);
    data[0]*=1e-9;
    vTimeStamps.push_back(data[0]);
    data.erase(data.begin());
    vToa.push_back(data);
  }
}