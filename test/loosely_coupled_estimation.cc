
// #include "Optimizer.h"

#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <random>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include <algorithm>

using namespace std;
class ToaEdgeTr : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
{

public:
    ToaEdgeTr() {}
    void computeError() override
    {
        // The first node is keyframe pose in camera frame, i.e., Tcw
        // The second node contains the transformation from world to vicon, i.e., Twv
        //  Landmark is in the vicon frame, i.e., Tvl
        const g2o::VertexSE3Expmap *v0 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::VertexSE3Expmap *v1 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[1]);
        Eigen::Vector3d landmark_tr = v1->estimate().map(_landmark);
        auto est_distance = (landmark_tr - v0->estimate().inverse().translation()).norm();
        _error[0] = est_distance - _measurement;
    }

    bool read(istream &is) override { return true; }
    bool write(ostream &os) const override { return true; }

    void setLandmark(const Eigen::Vector3d &landmark)
    {
        _landmark = landmark;
    }
    const Eigen::Vector3d &getLandmark() const
    {
        return _landmark;
    }
    void setMeasurement(double measurement) { _measurement = measurement; }

private:
    double _measurement;
    Eigen::Vector3d _landmark;
};

class FullPoseEdge : public g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap>
{
    // g2o::SE3Quat pose(qw, qx, qy, qz, x, y, z);
    // g2o::SE3Quat pose(qw, qx, qy, qz, x, y, z);
public:
    FullPoseEdge() {}
    void computeError() override
    {
        const g2o::VertexSE3Expmap *v0 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat delta = _measurement.inverse() * v0->estimate();
        _error = delta.log();
    }

    bool read(istream &is) override { return true; }
    bool write(ostream &os) const override { return true; }

    void setMeasurement(g2o::SE3Quat measurement) { _measurement = measurement; }

private:
    g2o::SE3Quat _measurement;
};

//NEW
int LoadPoses(const string strToaPath, vector<double> &TimeStamps,
              vector<vector<double>> &poses, bool bEurocStyle = true)
{
    // Load the ground truth with the format of time_stamp, x, y, z, qw, qx, qy, qz (euroc style)
    ifstream file;
    file.open(strToaPath);
    // Check if the file was opened successfully
    if (!file.is_open())
    {
        cout << "Failed to open file." << endl;
        return 1;
    }

    // Read each line of the CSV file
    std::string line;
    if (bEurocStyle)
    {
        // throwing away the first line
        getline(file, line);
    }

    while (std::getline(file, line))
    {

        std::vector<double> values;
        std::stringstream ss(line);
        double value;

        while (ss >> value)
        {
            values.push_back(value);
            if (bEurocStyle)
            {
                // Ignore the comma separator for Euroc style
                ss.ignore();
            }
        }

        // Add the values vector to the data vector
        if (!bEurocStyle)
        {
            // TUM style, the qw comes at last
            std::swap(values[4], values[7]);
        }
        TimeStamps.push_back(values[0]);
        values.erase(values.begin());
        poses.push_back(values);
    }

    // Close the file
    file.close();
    cout << "The" << strToaPath << "is loaded successfully!" << endl;
    return 0;
}

g2o::SE3Quat createRandomSE3Increment(double translation_stddev, double rotation_stddev)
{
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> translation_distribution(0.0, translation_stddev);
    normal_distribution<double> rotation_distribution(0.0, rotation_stddev);

    // Generate random translation
    Eigen::Vector3d translation(
        translation_distribution(gen),
        translation_distribution(gen),
        translation_distribution(gen));

    // Generate random Euler angles
    Eigen::Vector3d euler_angles(
        rotation_distribution(gen),
        rotation_distribution(gen),
        rotation_distribution(gen));

    // Convert Euler angles to quaternion
    Eigen::Quaterniond rotation = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX()) *
                                  Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *
                                  Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());

    // Combine translation and rotation into SE3Quat
    g2o::SE3Quat random_increment(rotation, translation);
    return random_increment;
}

Eigen::Vector3d createRandomLandmark(double translation_stddev)
{
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> translation_distribution(0.0, translation_stddev);

    // Generate random translation
    Eigen::Vector3d translation(
        translation_distribution(gen),
        translation_distribution(gen),
        translation_distribution(gen));
    return translation;
}

double genToANoise(double stddev)
{
    // Create a random number generator with a normal distribution
    // default_random_engine generator;
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> distribution(0.0, stddev);
    // Generate random values for the quaternion components
    double x = distribution(gen);
    return x;
}

double computeRMSE(const std::vector<g2o::Vector6d> &errors)
{
    int num_errors = errors.size();
    g2o::Vector6d squared_errors_sum = g2o::Vector6d::Zero();

    for (int i = 0; i < num_errors; ++i)
    {
        squared_errors_sum += errors[i].matrix().array().square().matrix();
    }

    double squared_errors_sum_avg = squared_errors_sum.sum() / (num_errors * 6);
    double rmse = std::sqrt(squared_errors_sum_avg);

    return rmse;
}

int optimizeGraph(int num_pose = 500, int num_landmarks = 1, double toaNoise = 0.1, bool fixed_landmarks = true, bool addToAFactors = true, bool generate_rnd_poses = false)
{
    // NEW
    // Gt poses are vicon with respect the the G world frame (Tgv)
    // so the distance between the Gtposes and landmark, would be the distance between the vicon and the landmark
    // However, the keyframe are poses of the body frame with respect to the world.
    // So we need to transform the GT poses to body frame, Tgb = Tgv * inverse(Tbv)
    Eigen::Matrix4d Tbv_matrix;
    Tbv_matrix << 0.33638, -0.01749, 0.94156, 0.06901,
        -0.02078, -0.99972, -0.01114, -0.02781,
        0.94150, -0.01582, -0.33665, -0.12395,
        0.0, 0.0, 0.0, 1.0;
    g2o::SE3Quat Tbv(Eigen::Matrix3d(Tbv_matrix.block<3, 3>(0, 0)), Eigen::Vector3d(Tbv_matrix.block<3, 1>(0, 3)));

    // loading ground truth poses and ORBSLAM pose estimates
    // TODO: there is no need to create GtPoses there is already vPose. Avoid to create many new if possible. Please also use the same or similar naming convention with lower letters
    vector<vector<double>> GtPoses;
    vector<double> gtPoseTimeStamps;
    int Gtposes_ind(0);
    string gtPath = "Datasets/EuRoC/V101/mav0/vicon0/data.csv";
    // TODO:  Please also use the same or similar naming convention with lower letters. Avoid variables not needed.
    vector<vector<double>> EstPoses;
    vector<double> EstPoseTimeStamps;
    // NOTE: load estimates from every frame not only key-frames
    string EstPath = "Results_baseline/euroc_inertial/f_V101_inertial.txt";

    // TODO: here you should load poses directly as vector<g2o::SE3Quat> in the format Tbw, use the associate.py function to obtain directly the gt pose for every estimated frame ( no asosciation has to be done here atm)
    LoadPoses(gtPath, gtPoseTimeStamps, GtPoses, true);
    LoadPoses(EstPath, EstPoseTimeStamps, EstPoses, false);
    // TODO: print as done below
    cout << "print the first vector of Gtposes:" << GtPoses[0][0] << GtPoses[0][1] << GtPoses[0][2] << GtPoses[0][3] << GtPoses[0][4] << GtPoses[0][5] << GtPoses[0][6] << endl;
    cout << "print the first vector of EstPoses:" << EstPoses[0][0] << EstPoses[0][1] << EstPoses[0][2] << EstPoses[0][3] << EstPoses[0][4] << EstPoses[0][5] << EstPoses[0][6] << endl;

    vector<Eigen::Vector3d> landmarks;
    landmarks.reserve(num_landmarks);
    // random initialization of the landmarks:
    if (!fixed_landmarks)
    {
        // These are position of Li wrt G : Li -> G
        for (int i = 0; i < num_landmarks; i++)
        {
            Eigen::Vector3d pos = createRandomLandmark(10);
            cout << "Landmark " << i << " created at position: " << pos.transpose() << endl;
            landmarks.push_back(pos);
        }
    }
    else
    {
        // manual initialization of the landmarks
        // TODO set landmarks to where you would actually place them inside the dataset (i.e., wrt vicon coord frame)
        std::vector<Eigen::Vector3d> fixed_landmarks = {{0, 0, 0}, {-2, 3, 4}, {8, -8.0, 9.0}};
        landmarks.assign(fixed_landmarks.begin(), fixed_landmarks.begin() + min(num_landmarks, 3));
    }

    g2o::SE3Quat gtPoseWG = createRandomSE3Increment(3, 30);
    cout << "Ground truth TWG (from G to W) " << gtPoseWG.toMinimalVector().transpose() << endl;

    // NEW: change the value of num_pose
    if (!generate_rnd_poses)
    {
        num_pose = EstPoses.size();
    }

    // TODO: as above you create too many variables start to be confusing. If you can reuse the existing. and vPose are ground truth poses already
    vector<g2o::SE3Quat> vGtPose;
    vector<g2o::SE3Quat> vPose;
    // TODO no need of measurments for example, delete
    vector<vector<double>> measurements;
    vPose.reserve(num_pose);


    // These are poses in W frame use this to transform from W -> C
    if (generate_rnd_poses)
    {
    // Create g2o::SE3Quat object with identity rotation and zero translation
    g2o::SE3Quat origin_pose;
    vPose.push_back(origin_pose);
        for (int i = 1; i < num_pose; i++)
        {
            double translation_stddev = 1; // Standard deviation for translation
            double rotation_stddev = .5;   // Standard deviation for rotation
            g2o::SE3Quat random_increment = createRandomSE3Increment(translation_stddev, rotation_stddev);
            g2o::SE3Quat p = random_increment * vPose[i - 1];
            cout << i << "th GT pose: " << random_increment.toMinimalVector().transpose() << endl;
            vPose.push_back(p);
        }
    }
    //NEW
    // TODO: here you should have the load functions! and use them to initialize vector<g2o::SE3Quat> vPose (this is the gt vector not the estimate in the example code) and another for the estimates
    else
    {
        for (int i = 0; i < num_pose; i++)
        {
            // for Orbslam mono-inertial case, the results are estimated as Twb
            g2o::SE3Quat p = g2o::SE3Quat(Eigen::Quaterniond(EstPoses[i][3], EstPoses[i][4], EstPoses[i][5], EstPoses[i][6]), Eigen::Vector3d(EstPoses[i][0], EstPoses[i][1], EstPoses[i][2]));
            vPose.push_back(p.inverse());
        }
    }
    cout << "print the size of vPose:" << vPose.size() << endl;
    cout << "pose_num:" << num_pose << endl;

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);


    g2o::VertexSE3Expmap *Twg = new g2o::VertexSE3Expmap();
    Twg->setId(-1);

    // initializing the tf with a possible initial value
     if (generate_rnd_poses)
    {
    Twg->setEstimate(createRandomSE3Increment(2, 10) * gtPoseWG);
    }
    else{
        // TODO: put the real transformation from VICON to the estimates origin. This is what TWG represents
    }

    if (addToAFactors)
    {
        optimizer.addVertex(Twg);
    }

    for (int i = 0; i < num_pose; i++)
    {

        g2o::SE3Quat currPose = vPose[i];
        // TODO: currPose is the GtPose already
        g2o::SE3Quat currGtPose;

        // TODO: this is better do it in the load or even before, here is a mess
        // find the Groud truth pose corresponding to the current (estimated) pose based on the timestamp
        while (gtPoseTimeStamps[Gtposes_ind] < EstPoseTimeStamps[i])
        {
            Gtposes_ind++;
            //TGv
            currGtPose = g2o::SE3Quat(Eigen::Quaterniond(GtPoses[Gtposes_ind][3], GtPoses[Gtposes_ind][4],
                                     GtPoses[Gtposes_ind][5], GtPoses[Gtposes_ind][6]), Eigen::Vector3d(GtPoses[Gtposes_ind][0], GtPoses[Gtposes_ind][1], GtPoses[Gtposes_ind][2]));
            //Tgb
            currGtPose = currGtPose*Tbv.inverse();
        }
        // TGv
        currGtPose = g2o::SE3Quat(Eigen::Quaterniond(GtPoses[Gtposes_ind][3], GtPoses[Gtposes_ind][4],
                                                     GtPoses[Gtposes_ind][5], GtPoses[Gtposes_ind][6]),
                                  Eigen::Vector3d(GtPoses[Gtposes_ind][0], GtPoses[Gtposes_ind][1], GtPoses[Gtposes_ind][2]));
        // Tgb
        currGtPose = currGtPose * Tbv.inverse();

        // add the keyframe vertex
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if (i == 0)
        {
            v->setFixed(true);
            v->setEstimate(currPose);
        }
        else
        {
            // TODO there is no need of having an edge with previous 20 poses with the real data, change this
            for (int j = i - 1; j >= max(i - 20, 0); j--)
            {

                g2o::SE3Quat random_increment = createRandomSE3Increment(0.5, 0.5);
                cout << "Random SE3 increment from node " << i << " to node " << j << ": " << random_increment.toMinimalVector().transpose() << endl;
                g2o::SE3Quat meas = random_increment * currPose * vPose[j].inverse();
                // TODO please do an if else, no need of random increment;
                if (!generate_rnd_poses)
                {
                    meas = currPose * vPose[j].inverse();
                }

                g2o::VertexSE3Expmap *prevPose = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(j));
                v->setEstimate(meas * prevPose->estimate());

                g2o::EdgeSE3 *e = new g2o::EdgeSE3();
                e->setMeasurement(meas);
                // TODO: add a variable for the information matrix of the poses
                e->setInformation(.01 * Eigen::Matrix<double, 6, 6>::Identity());
                e->setVertex(0, optimizer.vertex(j));
                e->setVertex(1, v);
                optimizer.addEdge(e);
            }
        }
        optimizer.addVertex(v);

        if (addToAFactors)
        {
            for (const auto &l : landmarks)
            {
                auto meas = genToANoise(toaNoise) + (gtPoseWG.map(l) - currPose.inverse().translation()).norm();
                // TODO: this should not be added with are still generating ToA measurments randomly from gt and currPose and gtPoseWg are all you should need for the generation atm
                // if (!generate_rnd_poses)
                // {
                //     // both landmark and Gt poses are the in G frame, so no need for transformation
                //     meas = genToANoise(toaNoise) + (l - currGtPose.translation()).norm();
                // }
                ToaEdgeTr *e = new ToaEdgeTr();
                e->setVertex(0, v);
                e->setVertex(1, Twg);
                e->setMeasurement(meas);
                e->setLandmark(l);
                // TODO: add a variable for the information matrix of the poses
                e->setInformation(.1 * Eigen::Matrix<double, 1, 1>::Identity());
                optimizer.addEdge(e);
            }
        }
    }

    // Optimize the graph
    cout << "Number of vertices: " << optimizer.vertices().size() << endl;
    cout << "Number of edges: " << optimizer.edges().size() << endl;
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(100);
    optimizer.computeActiveErrors();

    vector<g2o::Vector6d> errors;
    errors.reserve(vPose.size());

    for (int i; i < vPose.size(); i++)
    {
        g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i));

        cout << i << " - Ground Truth pose " << vPose[i].toMinimalVector().transpose() << endl;
        cout << i << " - Estimated pose " << v->estimate().toMinimalVector().transpose() << endl;
        cout << endl;
        errors.push_back((v->estimate() * vPose[i].inverse()).log());
    }

    cout << "RMSE Poses: " << computeRMSE(errors) << endl;
    cout << endl;

    // Printing the Transformation and it estimation
    if (addToAFactors)
    {
        g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(-1));
        cout << "Ground truth TWG: " << gtPoseWG.toMinimalVector().transpose() << endl;
        cout << "Estimated TWG: " << v->estimate().toMinimalVector().transpose() << endl;
        cout << "RMSE TWG: " << computeRMSE({(v->estimate() * gtPoseWG.inverse()).log()}) << endl;
        cout << endl;
    }
    else
    {
        cout << "ToA factors not added to optimizer" << endl;
    }
    // TODO: return the poses rmse and the Twg rmse for comapring multiple runs. you can return a value here or set a passed reference if you prefer.
    return 0;
}

int main(int argc, char *argv[])
{
    optimizeGraph();
    return 0;
}
