
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

void OptimizeToa(int frameID, std::vector<double> vToa,  g2o::SparseOptimizer& optimizer, bool bSim3 = true);
class ToaEdgeTr : public g2o::BaseBinaryEdge<1, double, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap> {

public:
    ToaEdgeTr() {}
    void computeError() override {  
        const g2o::VertexSE3Expmap* v0 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        const g2o::VertexSE3Expmap* v1 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[1]);
        Eigen::Vector3d landmark_tr;
        landmark_tr= v1->estimate().map(_landmark);
        const Eigen::Vector3d& translation = v0->estimate().translation();
        double dEst = (translation - landmark_tr ).norm();
        _error[0] = dEst - _measurement;
        // cout<<"This is the landmark unary: "<<_landmark<<endl;
        // cout<<"This is the translation: "<<translation<<endl;
        // cout<<"This is the measurement: "<<_measurement<<endl;
        // cout<<"dEst TOAEdge unary: "<<dEst<<endl;
        // cout<<"_error: "<<_error[0]<<endl;
        // cout<<"---------------------------------"<<endl;
        }


    bool read(istream& is) override { return true; }
    bool write(ostream& os) const override { return true; }

    void setLandmark(vector<double> landmark) { _landmark = Eigen::Vector3d(landmark[0], landmark[1], landmark[2]); }
    void setMeasurement(double measurement) { _measurement = measurement; }

private:
    double _measurement;
    Eigen::Vector3d _landmark;
};


class FullPoseEdge : public g2o::BaseUnaryEdge<6, g2o::SE3Quat, g2o::VertexSE3Expmap> {
// g2o::SE3Quat pose(qw, qx, qy, qz, x, y, z);
// g2o::SE3Quat pose(qw, qx, qy, qz, x, y, z);
public:
    FullPoseEdge() {}
    void computeError() override {  
        const g2o::VertexSE3Expmap* v0 = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        g2o::SE3Quat delta = _measurement.inverse() * v0->estimate();
        _error = delta.log();
        
        
        }


    bool read(istream& is) override { return true; }
    bool write(ostream& os) const override { return true; }

    void setMeasurement(g2o::SE3Quat measurement) { _measurement = measurement; }

private:
    g2o::SE3Quat _measurement;
};


///////////////////////////////////////////////////////////////////
///////////////////////////////////Main start/////////////////////
/////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
int main(int argc, char *argv[])
{
Eigen::Quaterniond q1(1,0,0, 0);
Eigen::Vector3d t1(1, 3, 2);
Eigen::Vector3d t1_noisy(1, 3, 2);
g2o::SE3Quat P_OC(q1, t1);
g2o::SE3Quat P_OC_noisy(q1, t1_noisy);

Eigen::Quaterniond q2 = Eigen::Quaterniond::UnitRandom();
Eigen::Vector3d t2 = Eigen::Vector3d::Random();
g2o::SE3Quat T_OW(q2, t2);

cout<<"This is the Transformation TOW"<<endl<<T_OW<<endl;


Eigen::Vector3d lm;
std::vector<std::vector<double>> l_vec = {{0, 1, 1}, {-2, 3, 4}, {8, -8.0, 9.0}, {-17.0, -20.0, 2.0}};
vector<double> meas(4);
vector<double> noise = {0.5, -0.5, 1, 0, 2};
for (int i = 0; i<4; i++)
    {
    lm = Eigen::Vector3d(l_vec[i][0], l_vec[i][1], l_vec[i][2]);   
    // cout<<"T_OW"<<T_OW<<endl;  
    // cout<<"T_OW mapping the lm"<<T_OW.map(lm)<<endl;
    // cout<<"---------------------------------"<<endl;
    meas[i] = (T_OW.map(lm) - t1).norm();//+noise[i]; 
    //  cout<<"Measurement is "<<meas[i]<<"this is i"<<i<<endl;
    }
    
    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    solver->setUserLambdaInit(1e-5);
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);
 

    //add vertex
    g2o::VertexSE3Expmap* v = new g2o::VertexSE3Expmap();
    v->setId(0);
    v->setEstimate(g2o::SE3Quat());
    optimizer.addVertex(v);  

    //add the transformation vertex
    g2o::VertexSE3Expmap* vt = new g2o::VertexSE3Expmap();
    vt->setId(1);
    vt->setEstimate(g2o::SE3Quat());
    // vt->setEstimate(T_OW);
    optimizer.addVertex(vt); 

    cout<<"thi is third"<<endl;
    //add the SE3 edge to the vertex 0
    FullPoseEdge* e1 = new FullPoseEdge();
    e1->setVertex(0, optimizer.vertex(0));
    e1->setMeasurement(P_OC_noisy);
    e1->setInformation(0.1*Eigen::Matrix<double, 6, 6>::Identity());
    optimizer.addEdge(e1);

    //add the binary edges between those two vertices
    for (int i = 0; i<4; i++){
    ToaEdgeTr* e2 = new ToaEdgeTr();
    e2->setVertex(0, v);
    e2->setVertex(1, vt);
    e2->setMeasurement(meas[i]);
    e2->setLandmark(l_vec[i]);
    e2->setInformation(0.000001*Eigen::Matrix<double, 1, 1>::Identity());
    optimizer.addEdge(e2);
    }




// // Print the values of vertices before optimization and after it
cout << "Vertex 0 estimate before optimization: " << v->estimate() << endl;
cout << "Vertex 1 estimate before optimization: " << vt->estimate() << endl;

    // Optimize the graph
    std::cout << "Number of vertices: " << optimizer.vertices().size() << std::endl;
    std::cout << "Number of edges: " << optimizer.edges().size() << std::endl;
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(20);
    optimizer.computeActiveErrors();


cout << "Vertex 0 estimate after optimization: " <<endl<<v->estimate()<< endl;
cout << "Vertex 1 estimate after optimization: " <<endl<<vt->estimate()<< endl;


return 1;
}

// void OptimizeToa(int frameID, std::vector<double> vToa,  g2o::SparseOptimizer& optimizer, bool bSim3 )
// {
//  optimizer.setVerbose(false);
// int lm_id = 0;     
// for (auto& m : vToa) {
//     lm_id ++;
//     // cout<<"this is the measurement: "<<m<<endl;
//     ToaEdgeUnary* e = new ToaEdgeUnary();
//     e->setMeasurement(m); 
//     e->setInformation(0.01*Eigen::Matrix<double, 1, 1>::Identity()); //TODO-msm here we assume the same uncertainty over all measurement 
//     e->setVertex(0,optimizer.vertex(frameID));
//     e->setLandmark(ToA::sBsPositions[lm_id-1]);
//     // e->setRobustKernel(new g2o::RobustKernelHuber());
//     optimizer.addEdge(e);   
// }

// };








