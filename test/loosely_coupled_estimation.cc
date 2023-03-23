
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
        // The second node contains the transformation from world to ground, i.e., Twg
        //  Landmark is in the vicon frame, i.e., Tgl
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

class ToaEdge : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap>
{

public:
    ToaEdge() {}
    void computeError() override
    {
        // Assumptions: gt: Tbg, nodes: Tbw, landmark: Twl = Twg * Tgl
        const g2o::VertexSE3Expmap *v0 = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        auto est_distance = (v0->estimate().inverse().translation() - _landmark).norm();
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

void addToaEdge(g2o::SparseOptimizer &optimizer, const Eigen::Vector3d &landmark, double measurement, g2o::VertexSE3Expmap *v, g2o::SE3Quat Twg, double cov)
{
    ToaEdge *e = new ToaEdge();
    e->setVertex(0, v);
    e->setMeasurement(measurement);
    e->setLandmark(Twg.map(landmark));
    e->setInformation(cov * Eigen::Matrix<double, 1, 1>::Identity());
    optimizer.addEdge(e);
}

void addToaEdgeTr(g2o::SparseOptimizer &optimizer, const Eigen::Vector3d &landmark, const double &measurement, g2o::VertexSE3Expmap *v0, g2o::VertexSE3Expmap *v1, double cov)
{
    ToaEdgeTr *e = new ToaEdgeTr();
    e->setVertex(0, v0);
    e->setVertex(1, v1);
    e->setMeasurement(measurement);
    e->setLandmark(landmark);
    e->setInformation(cov * Eigen::Matrix<double, 1, 1>::Identity());
    optimizer.addEdge(e);
}

// NEW
int LoadPoses(const string strToaPath, vector<g2o::SE3Quat> &gtPoses, vector<g2o::SE3Quat> &estPoses, const g2o::SE3Quat &calibTbv)
{
    // Load the ground truth with the format of time_stamp, x, y, z, qw, qx, qy, qz (euroc style)
    ifstream file;
    file.open(strToaPath);
    // Check if the file was opened successfully
    if (!file.is_open())
    {
        cout << "Failed to open file: " << strToaPath << endl;
        return 1;
    }

    // Read each line of the CSV file
    std::string line;

    while (std::getline(file, line))
    {

        std::vector<double> values;
        std::stringstream ss(line);
        double value;

        while (ss >> value)
        {
            values.push_back(value);
        }

        g2o::SE3Quat p;
        p = g2o::SE3Quat(Eigen::Quaterniond(values[7], values[5], values[6], values[4]), Eigen::Vector3d(values[1], values[2], values[3]));
        gtPoses.push_back(p); // Tgv

        int offset = 2 + 7 + 0; // 2 timestamps + first_pose + additional fields of gt

        p = g2o::SE3Quat(Eigen::Quaterniond(values[offset + 6], values[offset + 4], values[offset + 5], values[offset + 3]), Eigen::Vector3d(values[offset], values[offset + 1], values[offset + 2]));
        estPoses.push_back(p); // Twb
    }

    // Close the file
    file.close();
    cout << "Successfully loaded the" << strToaPath << " file!" << endl;
    return 0;
}

int LoadTransformedPoses(string path_to_gt, vector<g2o::SE3Quat> &vGtPose, vector<g2o::SE3Quat> &vEstPose, g2o::SE3Quat &Twg)
{
    Eigen::Matrix4d Tbv_matrix;
    Tbv_matrix << 0.33638, -0.01749, 0.94156, 0.06901,
        -0.02078, -0.99972, -0.01114, -0.02781,
        0.94150, -0.01582, -0.33665, -0.12395,
        0.0, 0.0, 0.0, 1.0;
    g2o::SE3Quat Tbv(Eigen::Matrix3d(Tbv_matrix.block<3, 3>(0, 0)), Eigen::Vector3d(Tbv_matrix.block<3, 1>(0, 3)));

    // Chanign the tbv to identity as the ground truth is already in the body frame
    Tbv = g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    // loading the ORBSLAM pose estimates
    int err = LoadPoses(path_to_gt, vGtPose, vEstPose, Tbv);
    if (err)
        return 1;
    // Obtain the Transformation Twg from the Ground truth to Odom World centres
    //  Twg = Twb*Tbg
    Twg = vEstPose[0] * Tbv * vGtPose[0].inverse();

    Eigen::Matrix4d align_matrix;
    align_matrix << 0.36852209, 0.24800063, 0.8959281, 0.83038384,
        0.09182906, -0.9687563, 0.23038806, 2.19203329,
        0.92507237, -0.00263085, -0.37978177, 1.05654092,
        0., 0., 0., 1.;
    g2o::SE3Quat align(Eigen::Matrix3d(align_matrix.block<3, 3>(0, 0)), Eigen::Vector3d(align_matrix.block<3, 1>(0, 3)));
    // Twg = align.inverse();
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

double computeAPERMSE(const std::vector<Eigen::Matrix<double, 4, 4>> &errors)
{
    int num_errors = errors.size();
    double squared_errors_sum = 0;
    // Eigen::Matrix<double, 4, 4> identity;
    Eigen::Matrix4d identity;
    identity.setIdentity();

    for (int i = 0; i < num_errors; ++i)
    {
        squared_errors_sum += pow((errors[i] - identity).norm(), 2);
    }

    double rmse = std::sqrt(squared_errors_sum / num_errors);
    return rmse;
}

double computeATERMSE(const std::vector<Eigen::Vector3d> &errors)
{
    int num_errors = errors.size();
    double squared_errors_sum = 0;
    Eigen::Vector3d identity;

    for (int i = 0; i < num_errors; ++i)
    {
        squared_errors_sum += pow((errors[i]).norm(), 2);
    }

    double rmse = std::sqrt(squared_errors_sum / num_errors);
    return rmse;
}

int optimizeGraph(int num_pose = 500, int num_landmarks = 6, double toaNoise = 0.1, bool fixed_landmarks = false, bool addToAFactors = true, bool generate_rnd_poses = false, bool transformation = true)
{

    vector<g2o::SE3Quat> vGtPose;
    vector<g2o::SE3Quat> vEstPose;
    g2o::SE3Quat gtPoseWG;

    if (!generate_rnd_poses)
    {
        // TODO: load file in a more general way (no hardcoded path). it does not work for me because the working dir is inside the test folder. Also, Datasets folder does not exits in the code.
        string assocPosesFile = "../associated_poses.txt";
        int err = LoadTransformedPoses(assocPosesFile, vGtPose, vEstPose, gtPoseWG);
        if (err)
            return 1;

        cout << "print the first vector of gtPose_raw: " << vGtPose[0].toMinimalVector().transpose() << endl;
        cout << "print the first vector of estPoses: " << vEstPose[0].toMinimalVector().transpose() << endl;
        cout << "size of the ground truth poses: " << vGtPose.size() << endl;
        cout << "size of the estimated poses: " << vEstPose.size() << endl;
        // NEW: change the value of num_pose
        num_pose = vEstPose.size();
        // check if the transformation is correct or not but printing the error between the estimation and the ground truth vector<g2o::Vector6d> errors;
        vector<Eigen::Matrix<double, 4, 4>> errors_full;
        vector<Eigen::Vector3d> errors_t;
        errors_full.reserve(num_pose);
        errors_t.reserve(num_pose);
        for (int i = 0; i < num_pose; i++)
        {
            // vEstPose[i]*vGtPose[i] does not result in zero error, but the other way around does!!!!???
            //  CLAUDIO: vEstPose[i]*vGtPose[i] should not be identity. The error of the estimation should be vEstPose[i]*gtPoseWG*vGtPose[i].inverse(), which is identity only in case of perfect odometry
            cout << "The error between estimation and ground truth is: " << endl
                 << (vEstPose[i].inverse() * gtPoseWG * vGtPose[i]).toMinimalVector().transpose() << endl;
            errors_full.push_back((vEstPose[i].inverse() * gtPoseWG * vGtPose[i]).to_homogeneous_matrix());
            errors_t.push_back((vEstPose[i].inverse() * gtPoseWG * vGtPose[i]).translation());
        }
        cout << "Baseline RMSE APE: " << computeAPERMSE(errors_full) << endl;
        cout << "Baseline RMSE ATE: " << computeATERMSE(errors_t) << endl;
    }
    else
    {
        gtPoseWG = createRandomSE3Increment(3, 30);
    }
    cout << "Ground truth Twg: " << gtPoseWG.toMinimalVector().transpose() << endl;

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
        std::vector<Eigen::Vector3d> fixed_landmarks = {{0, 0, 0}, {-6, 15, 5}, {10, -3, 8}};
        landmarks.assign(fixed_landmarks.begin(), fixed_landmarks.begin() + min(num_landmarks, static_cast<int>(fixed_landmarks.size())));
    }
    cout << "landmarks size: " << landmarks.size() << endl;

    // These are poses in W frame use this to transform from W -> C
    // Create g2o::SE3Quat object with identity rotation and zero translation
    g2o::SE3Quat origin_pose;
    if (generate_rnd_poses)
    {

        vGtPose.push_back(origin_pose);
        for (int i = 1; i < num_pose; i++)
        {
            double translation_stddev = 1; // Standard deviation for translation
            double rotation_stddev = .5;   // Standard deviation for rotation
            g2o::SE3Quat random_increment = createRandomSE3Increment(translation_stddev, rotation_stddev);
            g2o::SE3Quat p = random_increment * vGtPose[i - 1];
            cout << i << "th GT pose: " << random_increment.toMinimalVector().transpose() << endl;
            vGtPose.push_back(p);
        }
    }
    else
        origin_pose = vEstPose[0];

    cout << "Num of poses to estimate:" << num_pose << endl;

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
    Twg->setEstimate(createRandomSE3Increment(0, 0) * gtPoseWG);
    // Twg->setEstimate(origin_pose);
    if (addToAFactors)
        optimizer.addVertex(Twg);

    for (int i = 0; i < num_pose; i++)
    {

        g2o::SE3Quat currPose = vGtPose[i];
        // add the keyframe vertex
        g2o::VertexSE3Expmap *v = new g2o::VertexSE3Expmap();
        v->setId(i);
        if (i == 0)
        {
            v->setFixed(true);
            v->setEstimate(origin_pose);
        }
        else
        {
            g2o::SE3Quat meas;
            if (generate_rnd_poses)
            {
                g2o::SE3Quat random_increment = createRandomSE3Increment(0.05, 0.05);
                cout << "Random SE3 increment from node " << i << " to node " << i - 1 << ": " << random_increment.toMinimalVector().transpose() << endl;
                meas = random_increment * currPose * vGtPose[i - 1].inverse();
            }
            else
            {
                meas = vEstPose[i] * vEstPose[i - 1].inverse();
            }

            g2o::VertexSE3Expmap *prevPose = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i - 1));
            // v->setEstimate(meas * prevPose->estimate());
            // v->setEstimate(currPose);
            // v->setFixed(true);

            g2o::EdgeSE3 *e = new g2o::EdgeSE3();
            e->setMeasurement(meas);
            // TODO: add a variable for the information matrix of the poses
            e->setInformation(.001 * Eigen::Matrix<double, 6, 6>::Identity());
            e->setVertex(0, optimizer.vertex(i - 1));
            e->setVertex(1, v);
            // optimizer.addEdge(e);
        }
        optimizer.addVertex(v);

        if (addToAFactors)
        {
            for (const auto &l : landmarks)
            {
                // Measurement is the distance between the landmark and the GT
                // currposes is Tbg, and l is Tgl, so we just convert tgb->Tgb and subtract them and take norm as both of them are in the same frame (G)
                // However, in ToAEdgeTr, as the node is Twb, we need transformation, to transform the landmark to the world.
                auto meas = genToANoise(toaNoise) + (l - currPose.translation()).norm();
                if (transformation)
                    addToaEdgeTr(optimizer, l, meas, v, Twg, 0.001);
                else
                    // Add EdgeToa without transformation
                    addToaEdge(optimizer, l, meas, v, gtPoseWG, 0.001);
            }
        }
    }

    // Optimize the graph
    cout << "Number of vertices: " << optimizer.vertices().size() << endl;
    cout << "Number of edges: " << optimizer.edges().size() << endl;
    cout << endl;
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(30);
    optimizer.computeActiveErrors();

    vector<Eigen::Matrix<double, 4, 4>> errors;
    errors.reserve(vGtPose.size());
    vector<Eigen::Vector3d> errors_t;
    errors_t.reserve(vGtPose.size());

    cout << "vGtPose.size() " << vGtPose.size() << endl;
    for (int i = 0; i < vGtPose.size(); i++)
    {
        g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i));
        errors.push_back((v->estimate() * gtPoseWG * vGtPose[i]).to_homogeneous_matrix());
        errors_t.push_back((v->estimate() * gtPoseWG * vGtPose[i]).translation());
    }

    cout << "RMSE APE Poses: " << computeAPERMSE(errors) << endl;
    cout << "--------------------------------------------" << endl;
    cout << "RMSE ATE Poses: " << computeATERMSE(errors_t) << endl;
    cout << endl;

    if (addToAFactors)
    {
        g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(-1));
        cout << "Ground truth TWG: " << gtPoseWG.toMinimalVector().transpose() << endl;
        cout << "Estimated TWG: " << v->estimate().toMinimalVector().transpose() << endl;
        cout << "RMSE APE Twg: " << computeAPERMSE({(v->estimate() * gtPoseWG.inverse()).to_homogeneous_matrix()}) << endl;
        cout << "RMSE ATE Twg: " << computeATERMSE({(v->estimate() * gtPoseWG.inverse()).translation()}) << endl;
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
    // TODO: run optimize graph with different configuration, then save configuration and rmse result in a CSV file (with header possibly) that can be ported to a table or a plot in a easy way
    // create another function that maybe loads the runs config from a file or it's handwritten inside at the beginning and run the function in a for loop
    return optimizeGraph();
}