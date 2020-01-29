/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"
#include <nlopt.hpp>

#define MAXBUFSIZE  ((int) 1e6)

Eigen::MatrixXd readMatrix(const char *filename)
{
  int cols = 0, rows = 0;
  double buff[MAXBUFSIZE];

    // Read numbers from file into buffer.
  std::ifstream infile;
  infile.open(filename);
  while (! infile.eof())
  {
    std::string line;
    getline(infile, line);

    int temp_cols = 0;
    std::stringstream stream(line);
    while(! stream.eof())
      stream >> buff[cols*rows+temp_cols++];

      if (temp_cols == 0)
        continue;

      if (cols == 0)
        cols = temp_cols;

        rows++;
  }

  infile.close();

  rows--;

    // Populate matrix with numbers.
  Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
     for (int j = 0; j < cols; j++)
        result(i,j) = buff[ cols*i+j ];

  return result;
};


//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _endEffector)
  : mRobot(_robot),
    mEndEffector(_endEffector)
{
  assert(_robot != nullptr);
  assert(_endEffector != nullptr);

  int dof = mRobot->getNumDofs();

  mForces.setZero(dof);

  mKp.setZero();
  mKv.setZero();

  for (int i = 0; i < 3; ++i)
  {
    mKp(i, i) = 750.0;
    mKv(i, i) = 250.0;

    mKpOr(i, i) = 150.0;
    mKvOr(i, i) = 50; 
  }

  // Remove position limits
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);

  // Set joint damping
  for (int i = 0; i < dof; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);

  mP = readMatrix("/home/krang/SethResearch/19a-7DOFArm-OrientationControl-RESCLF_Aug/P_space_SpeedReg_Full.txt");
  mF = readMatrix("/home/krang/SethResearch/19a-7DOFArm-OrientationControl-RESCLF_Aug/F_space_SpeedReg_Full.txt");
  mG = readMatrix("/home/krang/SethResearch/19a-7DOFArm-OrientationControl-RESCLF_Aug/G_space_SpeedReg_Full.txt"); 

  mddqBodyRef = Eigen::VectorXd::Zero(6);

}

//==============================================================================
Controller::~Controller()
{
}
//==============================================================================
struct OptParams{
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};



//==============================================================================
void matprint(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      std::cout << A(i,j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

//==============================================================================
double optFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
  OptParams* optParams = reinterpret_cast<OptParams *>(my_func_data);
  Eigen::Matrix<double, 7, 1> X(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 7, 1> mGrad = optParams->P.transpose()*(optParams->P*X - optParams->b);
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5 * pow((optParams->P*X - optParams->b).norm(), 2));
}


//********************************************RES-CLF Optimization Parameters
struct OptParams_RESCLF {
  Eigen::MatrixXd L_G;
  Eigen::VectorXd L_F;
  Eigen::VectorXd V_x;
  double gamma;
  double relaxation;
};


double optFunc_RESCLF(const std::vector<double>& x, std::vector<double>& grad,
               void* my_func_data) {
  OptParams_RESCLF* optParams_RESCLF = reinterpret_cast<OptParams_RESCLF*>(my_func_data);
  // std::cout << "done reading optParams in optFunc_RESCLF" << std::endl;
  // Eigen::Matrix<double, 18, 1> X(x.data());
  size_t n = x.size();
  Eigen::VectorXd X = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; i++) X(i) = x[i];
  // std::cout << "done reading x" << std::endl;

  if (!grad.empty()) {
    Eigen::MatrixXd mGrad = 2 * X;
    mGrad(0,0) = 2*optParams_RESCLF->relaxation*X(0);
    // std::cout << "done calculating gradient in optFunc_RESCLF" << std::endl;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
    // std::cout << "done changing gradient cast in optFunc_RESCLF" << std::endl;
  }
  // std::cout << "about to return something from optFunc_RESCLF" << std::endl;
  double output = 0;
    for(int i = 1; i < n;i++){
      output = output + pow(X(i),2);
    }
    output = output + optParams_RESCLF->relaxation*pow(X(0),2);
    // std::cout << "Returning output" << std::endl;
  return output;
}

double constraintFunc_RESCLF1(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  OptParams_RESCLF* constParams = reinterpret_cast<OptParams_RESCLF*>(my_func_data);
  // std::cout << "done reading optParams in constraintFunc_RESCLF1" << std::endl;
  
  // std::cout << "size of constParams->L_G = " << constParams->L_G.rows() << "*" << constParams->L_G.cols() << std::endl;
  // double gamma = optParams->gamma;
  size_t n = x.size();
  // Eigen::Matrix<double, 8, 1> X(x.data());
  Eigen::VectorXd X = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; i++) X(i) = x[i];

  if (!grad.empty()) {
    Eigen::MatrixXd mGrad = Eigen::VectorXd::Zero(n);
    
    for(int i = 1;i < n ;i++){
      mGrad(i,0) = constParams->L_G(0,i-1);
    }
    
    mGrad(0,0) = -1;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
    
  }
  //   // if (!grad.empty()) {
    //  grad[0] = 1;
    //  grad[1] = 0;
    // }
  
  Eigen::Matrix<double,1 ,1> mResult;
  mResult = constParams->L_G * X.tail(3) + constParams->L_F + constParams->gamma * constParams->V_x - X.segment<1>(0);//constParams->relaxation*Eigen::VectorXd::Ones(1);
  // mResult = mResult.col(0) - X(0);
  
  double result;
  result = mResult(0,0);
  // std::cout << "Returning result" << std::endl;
  return result;

}

void constraintFunc(unsigned m, double* result, unsigned n, const double* x,
                    double* grad, void* f_data) {
  OptParams* constParams = reinterpret_cast<OptParams*>(f_data);
  // std::cout << "done reading optParams " << std::endl;

  if (grad != NULL) {
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        grad[i * n + j] = constParams->P(i, j);
      }
    }
  }
  // std::cout << "done with gradient" << std::endl;

  Eigen::MatrixXd X = Eigen::VectorXd::Zero(n);
  for (size_t i = 0; i < n; i++) X(i) = x[i];
  // std::cout << "done reading x" << std::endl;

  Eigen::VectorXd mResult;
  mResult = constParams->P * X - constParams->b;
  for (size_t i = 0; i < m; i++) {
    result[i] = mResult(i);
  }
  // std::cout << "done calculating the result"
}

void constraintFunc_RESCLF2(unsigned m, double* result, unsigned n, const double* x,
                    double* grad, void* f_data) {
  OptParams* constParams = reinterpret_cast<OptParams*>(f_data);
  // std::cout << "done reading optParams in constraintFunc_RESCLF2" << std::endl;

  // std::cout << "value of m = " << m << "\nvalue of n = " << n << std::endl;
  if (grad != NULL) {
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n-1; j++) {
        grad[i * n + j] = constParams->P(i, j);
      }
    }
  }
  // std::cout << "done with gradient" << std::endl;

  Eigen::MatrixXd X = Eigen::VectorXd::Zero(n);
  for (size_t i = 0; i < n; i++) X(i) = x[i];
  // std::cout << "done reading x" << std::endl;

  Eigen::VectorXd mResult;
  mResult = constParams->P * X.block<3,1>(1,0) - constParams->b;
  for (size_t i = 0; i < m; i++) {
    result[i] = mResult(i);
  }
  // std::cout << "done calculating the result" << std::endl;
}
//==============================================================================
void Controller::update(const Eigen::Vector3d& _targetPosition, const Eigen::Vector3d& _targetRPY)
{
  double wPos = 1, wOr = 1;

  using namespace dart;

  Eigen::VectorXd dq = mRobot->getVelocities();                 // n x 1

  Eigen::MatrixXd MM = mRobot->getMassMatrix();
  std::cout << "Size of MM = " << MM.rows() << "*" << MM.cols() << std::endl;
  // matprint(mass_matrix);
  Eigen::MatrixXd hh = mRobot->getCoriolisAndGravityForces();
  std::cout << "Size of hh = " << hh.rows() << "*" << hh.cols() << std::endl;



  // End-effector Position
  Eigen::Vector3d x = mEndEffector->getTransform().translation();
  Eigen::Vector3d dx = mEndEffector->getLinearVelocity();
  Eigen::Vector3d ddxref = -mKp*(x - _targetPosition) - mKv*dx;
  math::LinearJacobian Jv = mEndEffector->getLinearJacobian();       // 3 x n
  math::LinearJacobian dJv = mEndEffector->getLinearJacobianDeriv();  // 3 x n
  Eigen::Matrix<double, 3, 7> PPos = Jv;
  Eigen::Vector3d bPos = -(dJv*dq - ddxref);

  // End-effector Orientation
  Eigen::Quaterniond quat(mEndEffector->getTransform().rotation());
  double quat_w = quat.w(); 
  Eigen::Vector3d quat_xyz(quat.x(), quat.y(), quat.z());
  if(quat_w < 0) {quat_w *= -1.0; quat_xyz *= -1.0; }
  Eigen::Quaterniond quatRef(Eigen::AngleAxisd(_targetRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(_targetRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(_targetRPY(2), Eigen::Vector3d::UnitZ()));
  double quatRef_w = quatRef.w(); 
  Eigen::Vector3d quatRef_xyz(quatRef.x(), quatRef.y(), quatRef.z());
  if(quatRef_w < 0) { quatRef_w *= -1.0; quatRef_xyz *= -1.0; }
  Eigen::Vector3d quatError_xyz = quatRef_w*quat_xyz - quat_w*quatRef_xyz + quatRef_xyz.cross(quat_xyz);
  // double quatError_w = quat_w*quatRef_w - quat_xyz.dot(quatRef_xyz);
  Eigen::Vector3d w = mEndEffector->getAngularVelocity();
  // Eigen::Vector3d dwref = -mKpOr*quatError_xyz/(2*quatError_w) - mKvOr*w;
  Eigen::Vector3d dwref = -mKpOr*quatError_xyz - mKvOr*w;
  math::AngularJacobian Jw = mEndEffector->getAngularJacobian();       // 3 x n
  math::AngularJacobian dJw = mEndEffector->getAngularJacobianDeriv();  // 3 x n
  Eigen::Matrix<double, 3, 7> POr = Jw;
  Eigen::Vector3d bOr = -(dJw*dq - dwref);


  // Optimizer stuff
  nlopt::opt opt(nlopt::LD_MMA, 7);
  OptParams optParams;
  std::vector<double> ddq_vec(7);
  double minf;

  // Perform optimization to find joint accelerations 
  Eigen::MatrixXd P(PPos.rows() + POr.rows(), PPos.cols() );
  P << wPos*PPos,
       wOr*POr;
  
  Eigen::VectorXd b(bPos.rows() + bOr.rows(), bPos.cols() );
  b << wPos*bPos,
       wOr*bOr;
        
  optParams.P = P;
   
  optParams.b = b;
  
  opt.set_min_objective(optFunc, &optParams);
   
  opt.set_xtol_rel(1e-4);
  opt.set_maxtime(0.005);

  opt.optimize(ddq_vec, minf);
 
  Eigen::Matrix<double, 7, 1> ddq(ddq_vec.data()); 
 
  //torques
  Eigen::MatrixXd M = mRobot->getMassMatrix();                   // n x n
  Eigen::VectorXd Cg   = mRobot->getCoriolisAndGravityForces();        // n x 1
  mForces = M*ddq + Cg;

  // ************************Computing RES-CLF*************
    // std::cout << "Check 1" << std::endl;
  Eigen::MatrixXd invM = mRobot->getInvMassMatrix();

  Eigen::MatrixXd MEE = Jv*invM*Jv.transpose();
  Eigen::MatrixXd MEE_Inverse = MEE.inverse();

  Eigen::MatrixXd MOr = Jw*invM*Jw.transpose();
  Eigen::MatrixXd MOr_Inverse = MOr.inverse();

  // Eigen::VectorXd bodyTorques1 = Jv.transpose()*MEE_Inverse*(ddxref) + Cg;
  // std::cout << "Check 2" << std::endl;
  const int J_size = Jv.cols();
  const int MEE_size = MEE.rows();

  // std::cout << "Check 3" << std::endl;
  Eigen::Matrix<double, 6,7> J_Aug;
  J_Aug << Jv, Jw;

    // std::cout << "Check 4" << std::endl;
  Eigen::Matrix<double, 6,6> M_Aug = Eigen::MatrixXd::Zero(6,6);
  M_Aug.topLeftCorner(3,3) = MEE_Inverse;
  std::cout << "Check 5" << std::endl;
  M_Aug.bottomRightCorner(3,3) = MOr_Inverse;
  
  std::cout << "MEE_Inverse = \n" <<MEE_Inverse <<std::endl;
  std::cout << "MOr_Inverse = \n" <<MOr_Inverse <<std::endl;
  std::cout << "M_Aug = \n" <<M_Aug <<std::endl;
  // std::cout << "Check 6" << std::endl;
  Eigen::Matrix<double, 6, 1> x_des;
  x_des << ddxref - dJv*dq,dwref -dJw*dq;

  std::cout << "x_des = \n" << x_des <<std::endl;

  std::cout << "J_Aug = \n" << J_Aug <<std::endl;
   // std::cout << "Check 7" << std::endl;
  Eigen::VectorXd bodyTorques1 = J_Aug.transpose()*M_Aug*x_des +  Cg;

  std::cout << "bodyTorques from RESCLF = \n" << bodyTorques1 << std::endl;
   std::cout << "bodyTorques from default = \n" << mForces << std::endl;

  // Apply the joint space forces to the robot
  // mRobot->setForces(mForces);
   mRobot->setForces(bodyTorques1);
}

//==============================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const
{
  return mRobot;
}

//==============================================================================
dart::dynamics::BodyNode* Controller::getEndEffector() const
{
  return mEndEffector;
}

//==============================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/)
{
}

