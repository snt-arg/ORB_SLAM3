
// #include "Optimizer.h"

#include <complex>

#include <Eigen/StdVector>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include <random>
#include <chrono>

// Get the current time as a seed for the random number generator

// #include "G2oTypes.h"
// #include "Converter.h"

// #include <mutex>

// #include "OptimizableTypes.h"

// #include <Toa.h>

// using namespace ORB_SLAM3;
using namespace std;

void OptimizeToa(int frameID, vector<double> vToa, g2o::SparseOptimizer &optimizer, bool bSim3 = true);

g2o::SE3Quat addSE3Noise(const g2o::SE3Quat &se3, double trans_stddev, double rot_stddev);
Eigen::Quaterniond generateQuaternionNoise(double stddev);
double generateToaNoise(double stddev);
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
        auto est_distance = (landmark_tr-v0->estimate().translation()).norm();
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

int main(int argc, char *argv[])
{

    int num_pose = 10;
    int num_landmarks = 4;
    // manual initialization of the landmarks:
    //    vector<Eigen::Vector3d> landmarks = {{0, 1, 1}, {-2, 3, 4}, {8, -8.0, 9.0}, {-17.0, -20.0, 2.0}};\
    //random initialization of the landmarks:
    vector<Eigen::Vector3d> landmarks;
    landmarks.reserve(num_landmarks);

    // These are position of Li wrt G : Li -> G
    for (int i = 0; i < num_landmarks; i++)
    {
        Eigen::Vector3d pos = createRandomLandmark(10);
        cout << "Landmark " << i << " created at position: " << pos.transpose() << endl;
        landmarks.push_back(pos);
    }

    g2o::SE3Quat gtPoseWG = createRandomSE3Increment(50, 10);
    cout << "Ground truth TWG (from G to W) " << gtPoseWG.toMinimalVector().transpose() << endl;

    vector<g2o::SE3Quat> vPose;
    vector<vector<double>> measurements;
    vPose.reserve(num_pose);

    // Create g2o::SE3Quat object with identity rotation and zero translation
    g2o::SE3Quat origin_pose;
    vPose.push_back(origin_pose);

    // These are poses in W frame use this to transform from W -> C
    for (int i = 1; i < num_pose; i++)
    {
        double translation_stddev = 10; // Standard deviation for translation
        double rotation_stddev = 5;     // Standard deviation for rotation
        g2o::SE3Quat random_increment = createRandomSE3Increment(translation_stddev, rotation_stddev);
        g2o::SE3Quat p = random_increment * vPose[i - 1];
        cout << i << "th GT pose: " << random_increment.toMinimalVector().transpose() << endl;
        vPose.push_back(p);
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    // add the transformation vertex from G to W initialized as the identity (wrong)
    g2o::VertexSE3Expmap *Twg = new g2o::VertexSE3Expmap();
    Twg->setId(-1);
    Twg->setEstimate(origin_pose);
    optimizer.addVertex(Twg);

    for (int i = 0; i < num_pose; i++)
    {

        g2o::SE3Quat currPose = vPose[i];
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
            for (int j = i - 1; j >= 0; j--)
            {

                g2o::SE3Quat random_increment = createRandomSE3Increment(0.35, 0.3);
                cout << "Random SE3 increment from node " << i << " to node " << j << ": " << random_increment.toMinimalVector().transpose() << endl;
                g2o::SE3Quat meas = random_increment * currPose * vPose[j].inverse();

                g2o::VertexSE3Expmap *prevPose = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(j));
                v->setEstimate(meas * prevPose->estimate());

                g2o::EdgeSE3 *e = new g2o::EdgeSE3();
                e->setMeasurement(meas);
                e->setInformation(.01 * Eigen::Matrix<double, 6, 6>::Identity());
                e->setVertex(0, optimizer.vertex(j));
                e->setVertex(1, v);
                optimizer.addEdge(e);
            }
        }
        optimizer.addVertex(v);

        for (const auto &l : landmarks)
        {
            auto meas = genToANoise(0.5) + (gtPoseWG.map(l) - currPose.translation()).norm();
            ToaEdgeTr *e = new ToaEdgeTr();
            e->setVertex(0, v);
            e->setVertex(1, Twg);
            e->setMeasurement(meas);
            e->setLandmark(l);
            e->setInformation(0.5 * Eigen::Matrix<double, 1, 1>::Identity());
            optimizer.addEdge(e);
        }
    }

    // Optimize the graph
    cout << "Number of vertices: " << optimizer.vertices().size() << endl;
    cout << "Number of edges: " << optimizer.edges().size() << endl;
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(50);
    optimizer.computeActiveErrors();

    for (int i; i < vPose.size(); i++)
    {
        g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(i));

        cout << "Ground Truth pose " << i << ": " << vPose[i].toMinimalVector().transpose() << endl;
        cout << "Estimated pose " << i << ": " << v->estimate().toMinimalVector().transpose() << endl;
    }

    // Printing the Transformation and it estimation
    if (optimizer.vertices().find(-1) != optimizer.vertices().end())
    {
        g2o::VertexSE3Expmap *v = dynamic_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(-1));
        cout << "Ground truth TWG" << gtPoseWG.toMinimalVector().transpose() << endl;
        cout << "Estimated TWG" << v->estimate().toMinimalVector().transpose() << endl;
    }
    else
    {
        cout << "TWG Not added to optimizer" << endl;
    }

    return 0;
}
