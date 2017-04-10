/**
 * \file      miscellaneous-algorithms.hpp
 * \author    Mehdi Benallegue
 * \date       2013
 * \brief      Gathers many kinds of algorithms
 *
 *
 *
 */

#include <Eigen/Core>
#include <state-observation/tools/definitions.hpp>

#ifndef SOTSTATEOBSERVATIONTOOLSMISCELANEOUSALGORITHMS
#define SOTSTATEOBSERVATIONTOOLSMISCELANEOUSALGORITHMS

namespace sotStateObservation
{
  
  ///transforms a homogeneous matrix into 6d vector (position theta mu)
  inline void homogeneousMatrixToVector6(const Eigen::Transform<double, 3,Eigen::Affine>& M,
                                         Eigen::Ref< Eigen::Matrix<double,6,1> > v)
  {
    const Eigen::AngleAxis<double> a(M.linear());
    v.head<3>() = M.translation(); 
    v.tail<3>() = a.angle() * a.axis();
    return;
  }


  ///transforms a 6d vector (position theta mu) into a homogeneous matrix
  inline void vector6ToHomogeneousMatrix(const Eigen::Matrix<double, 6, 1>& v, 
                                         Eigen::Transform<double,3,Eigen::Affine>& M) {
    M.translation() = v.head<3>();
    double angle = v.tail<3>().squaredNorm();
    if (angle > stateObservation::cst::epsilonAngle * stateObservation::cst::epsilonAngle) {
      angle=sqrt(angle);
      M.linear() = Eigen::AngleAxisd(angle, v.tail<3>()/angle).toRotationMatrix();
    }
    else
      M.linear() = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    
      return;
  }
}

namespace stateObservation {
  namespace tools {
    template<typename Derived1, typename Derived2, typename Derived3>
    inline void derivate(const Eigen::MatrixBase<Derived1>& o1,
                         const Eigen::MatrixBase<Derived2>& o2,
                         Eigen::MatrixBase<Derived3>& o,
                         double dt) {
      o = (o2-o1)*(1/dt);
    }

  }
}


#endif //SOTSTATEOBSERVATIONTOOLSMISCELANEOUSALGORITHMS
