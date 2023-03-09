
#include "Optimizer.h"


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
#include "G2oTypes.h"
#include "Converter.h"

#include<mutex>

#include "OptimizableTypes.h"

#include<Toa.h>



using namespace ORB_SLAM3;
using namespace std;
class ToaEdgeUnary : public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap> {

public:
    ToaEdgeUnary() {}
    void computeError() override {  
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const Eigen::Vector3d& translation = v->estimate().translation();
        double dEst = (translation - _landmark ).norm();
        _error[0] = dEst - _measurement;
        cout<<"This is the landmark unary: "<<_landmark<<endl;
        cout<<"This is the translation: "<<translation<<endl;
        cout<<"This is the measurement: "<<_measurement<<endl;
        cout<<"dEst TOAEdge unary: "<<dEst<<endl;
        cout<<"_error: "<<_error[0]<<endl;
        cout<<"---------------------------------"<<endl;
        }


    bool read(istream& is) override { return true; }
    bool write(ostream& os) const override { return true; }

    void setLandmark(vector<double> landmark) { _landmark = Eigen::Vector3d(landmark[0], landmark[1], landmark[2]); }
    void setMeasurement(double measurement) { _measurement = measurement; }

private:
    double _measurement;
    Eigen::Vector3d _landmark;
};





// The Unary g2o edge class takes single vertex of type SIM3 and a measurement of type Vector3d
class TOASim3edge : public g2o::BaseUnaryEdge<1, double, g2o::VertexSim3Expmap> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TOASim3edge() {}

    void computeError() override {
        const g2o::VertexSim3Expmap* v = static_cast<const g2o::VertexSim3Expmap*>(_vertices[0]);
        Sophus::SE3f Tiw(v->estimate().rotation().cast<float>(), v->estimate().translation().cast<float>()/v->estimate().scale() );
        // cout<<"Tiw: "<<Tiw.matrix()<<endl;
        // cout<<"_landmark: "<<_landmark<<endl;
        double dEst = (Tiw.inverse() * _landmark).head<3>().norm();
        _error[0] = dEst - _measurement;
        cout<<"dEst: "<<dEst<<endl;
        cout<<"_error: "<<_error[0]<<endl;
        
    }

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
    void setLandmark(vector<double> landmark) { _landmark = Eigen::Vector4f(landmark[0], landmark[1], landmark[2], 1); }
    void setMeasurement(double measurement) { _measurement = measurement; }


//     void linearizeOplus() {

//     // Eigen::Matrix3d R = v->estimate().rotation().toRotationMatrix(); // get rotational part of node
//     // Eigen::Matrix3d J = R / norm; // compute Jacobian
//     // _jacobianOplusXi.block<1,3>(0,0) = -J * trans.transpose(); // fill Jacobian matrix
//     // _jacobianOplusXi.block<1,3>(0,3) = Eigen::Matrix<double,1,3>::Zero(); // zero out the Jacobian for the rotational part
//     // 
//     Eigen::Matrix<double, 1, 7> W = Eigen::Matrix<double, 1, 7>::Identity(1,7);
//     _jacobianOplusXi = W;
    
//   }

// void linearizeOplus() {
//     BaseUnaryEdge::linearizeOplus();
//     std::cout << "Jacobian matrix: " << std::endl << _jacobianOplusXi << std::endl;
// }

    private:
    double _measurement;
    Eigen::Vector4f _landmark;
};




///////////////////////////////////////////////////////////////////
///////////////////////////////////Main start/////////////////////
/////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
int main(int argc, char *argv[])
{
Eigen::Quaterniond q1(1,0,0, 0);
Eigen::Vector3d t1(.1, .2, 0.1);
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
Landmarks[0] << 0, 1, 1;
Landmarks[1] << -2, 3, 4;
Landmarks[2] << 8, -8.0, 9.0;
Landmarks[3] << -17.0, -20.0, 2.0;

std::vector<std::vector<double>> l_vec = {{0, 1, 1}, {-2, 3, 4}, {8, -8.0, 9.0}, {-17.0, -20.0, 2.0}};
vector<double> meas(4);
vector<double> noise = {0.5, -0.5, 1, 0, 2};
for (int i = 0; i<4; i++)
    {
     meas[i] = (Landmarks[i] - t1).norm();//+noise[i]; 
     cout<<"Measurement is "<<meas[i]<<"this is i"<<i<<endl;
    }
    
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
    // optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(false);


    // solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);


    

    //add vertex
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    // g2o::VertexSim3Expmap* v = new g2o::VertexSim3Expmap();
    // ORB_SLAM3::VertexSim3Expmap * vSim3 = new ORB_SLAM3::VertexSim3Expmap();

    v->setId(0);
    v->setEstimate(g2o::SE3Quat());
    // v->setEstimate(g2o::Sim3());
    // v->setMarginalized(false);
    // v->_fix_scale = true;
    optimizer.addVertex(v);  

        // Sophus::SE3f Tiw(v->estimate().rotation().cast<float>(), v->estimate().translation().cast<float>() / v->estimate().scale());
        // Eigen::Vector4f lm = Eigen::Vector4f(l_vec[0][0], l_vec[0][1], l_vec[0][2], 1);
        // cout<<"Tiw: "<<Tiw.matrix()<<endl;
        // cout<<"_landmark: "<<lm<<endl;
        // double dEst = (Tiw.inverse() * lm).head<3>().norm();
        // cout<<"dEst: "<<dEst<<endl;

    cout<<"hfsdsfhsdhfkjs hsdfsf ----"<<endl;       
    //convert the Landmarks to vector doubles
    //add edge  
    int j = 0;
    std::vector<double> vec;
    for (auto& m: meas){
    cout<<"this is m"<<m<<endl;
    // cout<<l_vec[j]<<endl;
    ToaEdgeUnary* e = new ToaEdgeUnary();
    // TOASim3edge* e = new TOASim3edge();
    e->setMeasurement(m); 
    e->setLandmark(l_vec[j]);
    e->setInformation(0.01*Eigen::Matrix<double, 1, 1>::Identity()); //TODO-msm here we assume the same uncertainty over all measurement 
    // e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
    e->setVertex(0,optimizer.vertex(0));
    
    // e->setRobustKernel(new g2o::RobustKernelHuber());
    optimizer.addEdge(e);  
    j++;
    } 

// Print the estimate of the VertexSE3Expmap with index 0
g2o::OptimizableGraph::Vertex* vv = optimizer.vertex(0);

g2o::VertexSE3Expmap* vSE3 = dynamic_cast<g2o::VertexSE3Expmap*>(vv);
// g2o::VertexSim3Expmap* vSE3 = dynamic_cast<g2o::VertexSim3Expmap*>(vv);
cout<<"this is the real pose"<< t1.transpose()<<endl;
cout << "Vertex 0 estimate before optimization: " << vSE3->estimate() << endl;
cout << "Vertex 0 estimate before optimization: " << vSE3->estimate().translation().transpose() << endl;

    // Optimize the graph
    optimizer.initializeOptimization();
    // optimizer.computeActiveErrors();
    optimizer.optimize(20);
    // optimizer.computeActiveErrors();


cout << "Vertex 0 estimate after optimization: " << vSE3->estimate().translation().transpose()<< endl;


return 1;
}











