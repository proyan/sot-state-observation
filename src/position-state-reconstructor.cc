#include <sstream>
#include <numeric>

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>

#include <dynamic-graph/factory.h>
#include <sot-state-observation/position-state-reconstructor.hh>
#include <sot-state-observation/tools/miscellaneous-algorithms.hpp>



namespace sotStateObservation
{
  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( PositionStateReconstructor,
                                       "PositionStateReconstructor" );

  PositionStateReconstructor::PositionStateReconstructor
  ( const std::string & inName):
    lastVector_(18),
    Entity(inName),
    dt_(0.005),
    inputSIN(0x0 ,
             "PositionStateReconstructor("+inName+")::input(vector)::sin"),
    inputFormatSIN(0x0 ,
                   "PositionStateReconstructor("+inName+")::input(flags)::inputFormat"),
    outputFormatSIN(0x0 ,
                    "PositionStateReconstructor("+inName+")::input(flags)::outputFormat"),
    outputSOUT( inputSIN<<inputFormatSIN<<outputFormatSIN,
                "PositionStateReconstructor("+inName+")::output(vector)::sout")
  {
    derivationNumberOfSamples_=1;
    distr_.resize(derivationNumberOfSamples_);

    signalRegistration (inputSIN);
    signalRegistration (inputFormatSIN);
    signalRegistration (outputFormatSIN);
    signalRegistration (outputSOUT);

    lastVector_.setZero();

    ::dynamicgraph::Vector initV(3);
    initV.setZero();

    linearVelocities_.push_back(initV);
    angularVelocities_.push_back(initV);
    linearAccelerations_.push_back(initV);
    angularAccelerations_.push_back(initV);

    dynamicgraph::Vector input;
    input <<0;
    dynamicgraph::Vector output;
    output<<0;

    inputSIN.setConstant(input);
    //        outputSOUT.setConstant(ouput);

    outputSOUT.setFunction(boost::bind(&PositionStateReconstructor::computeOutput,
                                       this, _1, _2));

    std::string docstring;

    //setCurrentValue
    docstring =
      "\n"
      "    Sets the current value of the vector.\n"
      "    Its the basis of the computation of \n"
      "    the derivations and integration of the output vector\n"
      "    (takes a 18x1 vector)\n"
      "\n";

    addCommand(std::string("setCurrentValue"),
               new
               dynamicgraph::command::Setter <PositionStateReconstructor,dynamicgraph::Vector>
               (*this, &PositionStateReconstructor::setCurrentValue, docstring));

    //setSampligPeriod
    docstring =
      "\n"
      "    Sets the sampling period.\n"
      "    takes a floating point number\n"
      "\n";

    addCommand(std::string("setSamplingPeriod"),
               new
               dynamicgraph::command::Setter <PositionStateReconstructor,double>
               (*this, &PositionStateReconstructor::setSampligPeriod, docstring));

    //getSampligPeriod
    docstring =
      "\n"
      "    Gets the sampling period.\n"
      "    gives a floating point number\n"
      "\n";

    addCommand(std::string("getSampligPeriod"),
               new
               dynamicgraph::command::Getter <PositionStateReconstructor,double>
               (*this, &PositionStateReconstructor::getSampligPeriod, docstring));

    //setFiniteDifferencesInterval
    docstring =
      "\n"
      "    Sets the number of sampling periods which \n"
      "    defines the finite difference derivation interval.\n"
      "    takes an integer number\n"
      "\n";

    addCommand(std::string("setFiniteDifferencesInterval"),
               new
               dynamicgraph::command::Setter <PositionStateReconstructor,int>
               (*this, &PositionStateReconstructor::setFiniteDifferencesInterval, docstring));

    //getFiniteDifferencesInterval
    docstring =
      "\n"
      "    Gets the number of sampling periods which \n"
      "    defines the finite difference derivation interval.\n"
      "    takes an integer number\n"
      "\n";

    addCommand(std::string("getFiniteDifferencesInterval"),
               new
               dynamicgraph::command::Getter <PositionStateReconstructor,int>
               (*this, &PositionStateReconstructor::getFiniteDifferencesInterval, docstring));

    //setLastVector
    docstring =
      "\n"
      "    Sets lastVector_ \n"
      "\n";

    addCommand(std::string("setLastVector"),
               new
               dynamicgraph::command::Setter <PositionStateReconstructor,dynamicgraph::Vector>
               (*this, &PositionStateReconstructor::setLastVector, docstring));



  }

  PositionStateReconstructor::~PositionStateReconstructor()
  {
  }

  stateObservation::Vector PositionStateReconstructor::averageDistribution(const unsigned n)
  {
    vec_.resize(n);
    vec_.setOnes();
    vec_=double(1./double(n))*vec_;
    return vec_;
  }

  stateObservation::Vector PositionStateReconstructor::gaussianDistribution(const unsigned n)
  {
    double mean=0;
    double stddev=std::sqrt((n-mean)*(n-mean)/4.6); // the first element of the window correspond to 10% of the last element.

    vec_.resize(n);
    double sum=0.;
    for(int i=0; i<vec_.size();++i)
      {
        vec_[i]=(1./(stddev*std::sqrt(6.28)))*std::exp(-0.5*((i-mean)/stddev)*((i-mean)/stddev));
        sum+=vec_[i];
      }
    vec_=(1./sum)*vec_;
    return vec_;
  }

  dynamicgraph::Vector& PositionStateReconstructor::computeOutput
  (dynamicgraph::Vector & output, const int& inTime)
  {

    dynamicgraph::Vector inputSig (inputSIN(inTime));

    dynamicgraph::sot::Flags inputFormat (inputFormatSIN(inTime));
    dynamicgraph::sot::Flags outputFormat (outputFormatSIN(inTime));

    dynamicgraph::Vector input (18);

    size_t inputIndex = 0;
    for (size_t i=0; i<6; ++i)
      {
        if (inputFormat(i))
          {
            input.segment<3>(3*i) = inputSig.segment<3>(inputIndex);
            inputIndex += 3;
          }
        else
          {
            input.segment<3>(3*i).setZero();
          }

      }

    size_t outputIndex = 0;
    size_t outputSize = 0;

    // Update distribution for filtered derivations
    distr_.resize(derivationNumberOfSamples_);
    distr_=gaussianDistribution(derivationNumberOfSamples_);

    /////////////////////
    //Linear position
    Eigen::Vector3d position;
    bool posSet=true;

    if (inputFormat(0))
      {
        position = input.segment<3>(0);
      }
    else
      {
        if (inputFormat(2))
          {
            position = lastVector_.head<3>()
              + dt_*input.segment<3>(6);
          }
        else
          {
            if (inputFormat(4))
              position = lastVector_.head<3>()
                + dt_*lastVector_.segment<3>(6)
                + 0.5*dt_*dt_*input.segment<3>(12);
            else
              {
                position.setZero();
                posSet=false;
              }
          }
      }
    if (outputFormat(0))
      {
        if (posSet)
          {
            outputSize +=3;
            output.resize(outputSize,false);
            output.segment<3>(outputIndex) = position;
            outputIndex +=3;
          }
        else
          throw std::runtime_error
            ("There is nothing to reconstruct the position !");
      }

    /////////////////////
    //orientation
    Eigen::Vector3d orientation;
    bool oriSet=true;

    if (inputFormat(1))
      orientation = input.segment<3>(3);
    else
      {
        if (inputFormat(3))
          {
            stateObservation::Vector velocity(input.segment<3>(9));

            stateObservation::Vector lastOrientation(lastVector_.segment<3>(3));

            stateObservation::Quaternion
              dr(stateObservation::kine::rotationVectorToAngleAxis(velocity*dt_));

            stateObservation::AngleAxis
              r(dr* stateObservation::kine::rotationVectorToAngleAxis(lastOrientation));

            orientation=(r.angle()*r.axis());
          }
        else
          {
            if (inputFormat(5))
              {
                stateObservation::Vector
                  velocity(dt_*input.segment<3>(15)+lastVector_.segment<3>(9));
                
                stateObservation::Vector
                  lastOrientation(lastVector_.segment<3>(3));
                
                stateObservation::Quaternion
                  dr (stateObservation::kine::rotationVectorToAngleAxis
                      (velocity*dt_));

                stateObservation::AngleAxis
                  r (dr* stateObservation::kine::rotationVectorToAngleAxis (lastOrientation));

                orientation= (r.angle()*r.axis());
              }
            else
              {
                orientation.setZero();
                oriSet=false;
              }
          }
      }
    if (outputFormat(1))
      {
        if (oriSet)
          {
            outputSize +=3;
            output.resize(outputSize,false);
            output.segment<3>(outputIndex) = orientation;
            outputIndex +=3;
          }
        else
          throw std::runtime_error
            ("There is nothing to reconstruct the orientation !");
      }

    /////////////////////
    //Linear velocity
    Eigen::Vector3d linearVelocity;
    Eigen::Vector3d curfddLinVel;
    bool linVelSet=true;

    if (inputFormat(2))
      linearVelocity = input.segment<3>(6);
    else
      {
        if (inputFormat(0))
          {
            stateObservation::tools::derivate
              (lastVector_.head<3>(),input.head<3>(),curfddLinVel, dt_);

            linearVelocities_.push_back(curfddLinVel);

            while (linearVelocities_.size()>derivationNumberOfSamples_)
              linearVelocities_.pop_front();

            //                linearVelocity = 1.0 /linearVelocities_.size()*
            //                    std::accumulate(linearVelocities_.begin(),
            //                        linearVelocities_.end(),zero);

            int i=0; linearVelocity.setZero();
            for (iterator=linearVelocities_.begin(); iterator != linearVelocities_.end(); ++iterator)
              {
                linearVelocity+=distr_[i]*(*iterator);
                ++i;
              }
          }
        else
          {
            if (inputFormat(4))
              linearVelocity= lastVector_.segment<3>(6)
                + dt_*input.segment<3>(12);
            else
              {
                linearVelocity.setZero();
                linVelSet=false;
              }
          }
      }
    if (outputFormat(2))
      {
        if (linVelSet)
          {
            outputSize +=3;
            output.resize(outputSize,false);
            output.segment<3>(outputIndex) = linearVelocity;
            outputIndex +=3;
          }
        else
          throw std::runtime_error
            ("There is nothing to reconstruct the linear velocity !");
      }

    /////////////////////
    //Angular velocity
    Eigen::Vector3d angularVelocity;
    Eigen::Vector3d curfddAngVel;
    bool angVelSet = true;

    if (inputFormat(3))
      angularVelocity= input.segment<3>(9);
    else
      {
        if (inputFormat(1))
          {
            curfddAngVel= stateObservation::kine::derivateRotationFD
              (lastVector_.segment<3>(3), input.segment<3>(3), dt_);

            angularVelocities_.push_back(curfddAngVel);

            while (angularVelocities_.size()>derivationNumberOfSamples_)
              angularVelocities_.pop_front();

            //                angularVelocity = 1.0 / angularVelocities_.size() *
            //                    std::accumulate( angularVelocities_.begin(),
            //                        angularVelocities_.end(),zero) ;

            int i=0; angularVelocity.setZero();
            for (iterator=angularVelocities_.begin(); iterator != angularVelocities_.end(); ++iterator)
              {
                angularVelocity+=distr_[i]*(*iterator);
                ++i;
              }
          }
        else
          {
            if (inputFormat(5))
              angularVelocity= lastVector_.segment<3>(9)
                + dt_*input.segment<3>(15);
            else
              {
                angularVelocity.setZero();
                angVelSet=false;
              }
          }
      }
    if (outputFormat(3))
      {
        if (angVelSet)
          {
            outputSize +=3;
            output.resize(outputSize,false);
            output.segment<3>(outputIndex) = angularVelocity;
            outputIndex +=3;
          }
        else
          throw std::runtime_error
            ("There is nothing to reconstruct the angular velocity !");
      }

    /////////////////////
    //Linear acceleration
    Eigen::Vector3d linearAcceleration;
    Eigen::Vector3d curfddLinAcc;
    bool linAccSet = true;

    if (inputFormat(4))
      linearAcceleration= input.segment<3>(12);
    else
      {
        if (linVelSet)
          {
            stateObservation::tools::derivate
              (lastVector_.segment<3>(6), linearVelocity,curfddLinAcc, dt_);

            linearAccelerations_.push_back(curfddLinAcc);

            while (linearAccelerations_.size()>derivationNumberOfSamples_)
              linearAccelerations_.pop_front();

            //                linearAcceleration = 1.0 / linearAccelerations_.size() *
            //                    std::accumulate(linearAccelerations_.begin(),
            //                        linearAccelerations_.end(), zero);

            int i=0; linearAcceleration.setZero();
            for (iterator=linearAccelerations_.begin(); iterator != linearAccelerations_.end(); ++iterator)
              {
                linearAcceleration+=distr_[i]*(*iterator);
                ++i;
              }
          }
        else
          {
            linearAcceleration.setZero();
            linAccSet=false;
          }
      }
    if (outputFormat(4))
      {
        if (linAccSet)
          {
            outputSize +=3;
            output.resize(outputSize,false);
            output.segment<3>(outputIndex) = linearAcceleration ;
            outputIndex +=3;
          }
        else
          throw std::runtime_error
            ("There is nothing to reconstruct the linear acceleration !");
      }

    /////////////////////
    //Angular acceleration
    Eigen::Vector3d angularAcceleration;
    Eigen::Vector3d curfddAngAcc;
    bool angAccSet = true;

    if (inputFormat(5))
      angularAcceleration = input.segment<3>(15);
    else
      {
        if (angVelSet)
          {
            stateObservation::tools::derivate
              (lastVector_.segment<3>(9), angularVelocity,curfddAngAcc, dt_);

            angularAccelerations_.push_back(curfddAngAcc);

            while (angularAccelerations_.size()>derivationNumberOfSamples_)
              angularAccelerations_.pop_front();

            //                    angularAcceleration = 1.0 / angularAccelerations_.size() *
            //                        std::accumulate( angularAccelerations_.begin(),
            //                           angularAccelerations_.end(), zero);

            int i=0; angularAcceleration.setZero();
            for (iterator=angularAccelerations_.begin(); iterator != angularAccelerations_.end(); ++iterator)
              {
                angularAcceleration+=distr_[i]*(*iterator);
                ++i;
              }
          }
        else
          {
            angularAcceleration.setZero();
            angAccSet=false;
          }
      }
    if (outputFormat(5))
      {
        if (angAccSet)
          {
            outputSize +=3;
            output.resize(outputSize,false);
            output.segment<3>(outputIndex) = angularAcceleration;
            outputIndex +=3;
          }
        else
          throw std::runtime_error
            ("There is nothing to reconstruct the angular acceleration !");
      }

    lastVector_.segment<3>(0) =position;
    lastVector_.segment<3>(3)= orientation;
    lastVector_.segment<3>(6)= linearVelocity;
    lastVector_.segment<3>(9)= angularVelocity;
    lastVector_.segment<3>(12)= linearAcceleration;
    lastVector_.segment<3>(15)= angularAcceleration;

    return output;

  }
}
