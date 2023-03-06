
// #include "Optimizer.h"


// #include <complex>

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
#include "OptimizableTypes.h"
// #include<Toa.h>
using namespace ORB_SLAM3;
using namespace std;


class UnaryEdgeSim3 : public g2o::BaseUnaryEdge<1, double, g2o::VertexSim3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  UnaryEdgeSim3() {}

  void computeError()
  {
    const g2o::VertexSim3Expmap* v = static_cast<const g2o::VertexSim3Expmap*>(_vertices[0]);

    // The measurement is a scalar, stored in the _measurement variable
    const double& x = _measurement;

    // Compute the predicted measurement
    _error[0] = x - v->estimate().translation().norm();
  }

  void setMeasurement(double m) { _measurement = m; }

  virtual bool read(std::istream& is) { /* implementation omitted */ }
  virtual bool write(std::ostream& os) const { /* implementation omitted */ }
};

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
    private:
    double _measurement;
    Eigen::Vector4f _landmark;
};



class GpsSim3edge : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSim3Expmap> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GpsSim3edge() {}

    void computeError() override {
        const g2o::VertexSim3Expmap* v = static_cast<const g2o::VertexSim3Expmap*>(_vertices[0]);
        _error = v->estimate().translation() - _measurement;
        cout<<"_error: "<<_error<<endl;
    }

  virtual bool read(std::istream& is) { return true; }
  virtual bool write(std::ostream& os) const { return true; }

    // void setLandmark(vector<double> landmark) { _landmark = Eigen::Vector4f(landmark[0], landmark[1], landmark[2], 1); }
    void setMeasurement(vector<double> measurement) { _measurement = Eigen::Vector3d(measurement[0], measurement[1], measurement[2]); }
    private:
    Eigen::Vector3d _measurement;
};
///////////////////////////////////////////////////////////////////
///////////////////////////////////Main start/////////////////////
/////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
int main(int argc, char *argv[])
{
Eigen::Quaterniond q1(1,0,0, 0);
Eigen::Vector3d t1(3, -4, 7);
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

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolverX::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); 
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // Add the vertices to the graph
  g2o::VertexSim3Expmap* v1 = new g2o::VertexSim3Expmap();
  v1->setEstimate(g2o::Sim3());
  v1->setId(0);
  v1->setFixed(true);
  v1->setMarginalized(false);
  v1->_fix_scale = true;
  optimizer.addVertex(v1);



  g2o::VertexSim3Expmap* v2 = new g2o::VertexSim3Expmap();

  v2->setEstimate(g2o::Sim3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(3,7,5), 1.1));
  v2->setId(1);
  v2->setFixed(false);
  optimizer.addVertex(v2);

  // Add the edges to the graph
  // UnaryEdgeSim3* e1 = new UnaryEdgeSim3();
  // e1->setMeasurement(0);
  // e1->setVertex(0, v1);
  // e1->information() = 0.01*Eigen::Matrix<double,1,1>::Identity();
  // optimizer.addEdge(e1);

  // UnaryEdgeSim3* e2 = new UnaryEdgeSim3();
  // e2->setMeasurement(10000);
  // e2->setVertex(0, v2);
  // e1->information() = 0.01*Eigen::Matrix<double,1,1>::Identity();
  // optimizer.addEdge(e2);


    g2o::Sim3 sim3_1(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1, 1, 1), 1.0);
    g2o::Sim3 sim3_2(Eigen::Matrix3d::Identity(), Eigen::Vector3d(5,10,7), 1.1);
    g2o::Sim3 edge_sim3 = sim3_2.inverse() * sim3_1;
    cout<<"edge_sim3: "<<edge_sim3<<endl;
    cout<<"edge_sim3: "<<edge_sim3.translation().transpose()<<endl;

    std::vector<Eigen::Vector3d> Landmarks(4);
    // Initialize the list with values
    Landmarks[0] << 0, 1, 1;
    Landmarks[1] << -2, 3, 4;
    Landmarks[2] << 8, -8.0, 9.0;
    Landmarks[3] << -17.0, -20.0, 2.0;
    vector<double> meas(4);
    for (int i = 0; i < 4; i++)
    {
        meas[i] = sim3_2.map(Landmarks[i]).norm();
        cout<<"meas is:   "<<meas[i]<<endl;
    }

  int i = 0;
  for (auto& m : meas){
    TOASim3edge* e = new TOASim3edge();
    e->setMeasurement(m);
    e->setLandmark(vector<double>{Landmarks[i](0), Landmarks[i](1), Landmarks[i](2)});
    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
    e->information() = 0.01*Eigen::Matrix<double,1,1>::Identity();
    optimizer.addEdge(e);
    i++;
  }; 

  GpsSim3edge* e4 = new GpsSim3edge();
  e4->setMeasurement(vector<double>{5, 10, 7});
  e4->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
  e4->information() = 0.001*Eigen::Matrix<double,3,3>::Identity();
  optimizer.addEdge(e4);

  g2o::EdgeSim3* e3 = new g2o::EdgeSim3();
  const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();
  e3->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
  e3->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(1)));
  e3->information() = 1*matLambda;
  e3->setMeasurement(edge_sim3);
  optimizer.addEdge(e3);


  // Optimize the graph
  optimizer.initializeOptimization();
  optimizer.optimize(10);

  // Print the results
  std::cout << "v1 estimate: " << v1->estimate() << std::endl;
  std::cout << "v2 estimate: " << v2->estimate().translation() << std::endl;


return 1;
}











