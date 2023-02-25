
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
// #include "G2oTypes.h"
// #include "Converter.h"

// #include<mutex>

// #include "OptimizableTypes.h"

#include<Toa.h>
// #include <Eigen/Core>
// #include <Eigen/Geometry>
// #include <g2o/types/slam3d/types_slam3d.h>



using namespace ORB_SLAM3;
using namespace std;

class ToaEdgeUnary : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap> {
public:
    ToaEdgeUnary() {}
    void computeError() override {
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const Eigen::Vector3d& nodeP = v->estimate().translation();
        double dEst = (nodeP - _landmark).norm();
        _error[0] = dEst - _measurement;
    }
    // void linearizeOplus() override {
    //     const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    //     const Eigen::Vector3d& nodeP = v->estimate().translation();
    //     double dEst = (nodeP - _landmark).norm();
    //     Eigen::Matrix<double, 1, 3> j1 = (nodeP - _landmark) / dEst;
    //     _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
    //     _jacobianOplusXi.segment<3>(0) = j1 * v->estimate().rotation().toRotationMatrix();
    // }

    bool read(std::istream& is) override {
        is >> _measurement;
        is >> _landmark(0);
        is >> _landmark(1);
        is >> _landmark(2);
        return true;
    }
    bool write(std::ostream& os) const override {
        os << _measurement << " ";
        os << _landmark(0) << " ";
        os << _landmark(1) << " ";
        os << _landmark(2) << " ";
        return true;
    }

    // bool read(istream& is) override { return true; }
    // bool write(ostream& os) const override { return true; }
    void setLandmark(vector<double> landmark) { _landmark = Eigen::Vector3d(landmark[0], landmark[1], landmark[2]); }
    void setMeasurement(double measurement) { _measurement = measurement; }

private:
    double _measurement;
    Eigen::Vector3d _landmark;
};


static void OptimizeToa(int frameID, std::vector<double> vToa,  g2o::SparseOptimizer& optimizer)
{
int lm_id = 0;       
for (auto& m : vToa) {
    lm_id ++;
    // g2o::VertexXYZ* e = new g2o::EdgeSE3XYZ();
    ToaEdgeUnary* e = new ToaEdgeUnary();
    e->setMeasurement(m); 
    e->setInformation(0.1*Eigen::Matrix<double, 1, 1>::Identity()); //TODO-msm here we assume the same uncertainty over all measurement 
    e->setVertex(0,optimizer.vertex(frameID));
    e->setLandmark(ToA::sBsPositions[lm_id-1]);
    // e->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(e);   
}

};
///////////////////////////////////////////////////////////////////
///////////////////////////////////Main start/////////////////////
/////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
int main(int argc, char *argv[])
{
Eigen::Quaterniond q1(1,0,0, 0);
Eigen::Vector3d t1(5, 5, 5);
g2o::SE3Quat pose1(q1, t1);

Eigen::Quaterniond q2 = Eigen::Quaterniond::UnitRandom();
Eigen::Vector3d t2(10, 20,15);
g2o::SE3Quat pose2(q2, t2);

Eigen::Quaterniond q3 = Eigen::Quaterniond::UnitRandom();
Eigen::Vector3d t3 = Eigen::Vector3d::Random();
g2o::SE3Quat pose3(q3, t3);

vector<g2o::SE3Quat> Poses;
Poses.push_back(pose1);
Poses.push_back(pose2);
Poses.push_back(pose3); 

//calculate measurements 

std::vector<Eigen::Vector3d> Landmarks(4);

// Initialize the list with values
Landmarks[0] << 6.0, 3.0, 5.0;
Landmarks[1] << 4.0, 5.0, 6.0;
Landmarks[2] << 7.0, -8.0, 9.0;
Landmarks[3] << -7.0, -12.0, 2.0;

std::vector<std::vector<double>> l_vec = {{6.0, 3.0, 5.0}, {4.0, 5.0, 6.0}, {7.0, -8.0, 9.0}, {-7.0, -12.0, 2.0}};
vector<double> meas(4);
vector<double> noise = {0.5, -0.5, 1, 0, 2};
for (int i = 0; i<4; i++)
    {
     meas[i] = (Landmarks[i] - t1).norm()+noise[i]; 
     cout<<"Measurement is "<<meas[i]<<"this is i"<<i<<endl;
    }
    
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

    //add vertex
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    v->setId(0);
    v->setEstimate(g2o::SE3Quat());
    optimizer.addVertex(v);  

//     int ind = -1;
//    cout<<"before----"<<endl;  
    // for (auto& l: Landmarks){
    //     ind++;
    //     for (int i = 0; i<3; i++){
    //         // l_vec[ind][i] = l[]; 
    //         cout<<l(i);}
    //         }
    cout<<"hfsdsfhsdhfkjs hsdfsf ----"<<endl;       
    //convert the Landmarks to vector doubles
    //add edge  
    int j = 0;
    std::vector<double> vec;
    for (auto& m: meas){
    cout<<"this is m"<<m<<endl;
    // cout<<l_vec[j]<<endl;
    ToaEdgeUnary* e = new ToaEdgeUnary();
    e->setMeasurement(m); 
    e->setLandmark(l_vec[j]);
    e->setInformation(0.1*Eigen::Matrix<double, 1, 1>::Identity()); //TODO-msm here we assume the same uncertainty over all measurement 
    e->setVertex(0,optimizer.vertex(0));
    
    e->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(e);  
    j++;
    } 

// Print the estimate of the VertexSE3Expmap with index 0
g2o::OptimizableGraph::Vertex* vv = optimizer.vertex(0);
g2o::VertexSE3Expmap* vSE3 = dynamic_cast<g2o::VertexSE3Expmap*>(vv);
cout<<"this is the real pose"<< t1.transpose()<<endl;
cout << "Vertex 0 estimate before optimization: " << vSE3->estimate().translation().transpose() << endl;

    // Optimize the graph
optimizer.initializeOptimization();
optimizer.optimize(10);


cout << "Vertex 0 estimate after optimization: " << vSE3->estimate().translation().transpose() << endl;


return 1;
}











