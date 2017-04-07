/*
 *  Copyright 2014 CNRS
 *
 *  Alexis MIFSUD
 */

#ifndef SOT_DYNAMIC_GRAPH_INPUT_RECONSTRUCTOR
#define SOT_DYNAMIC_GRAPH_INPUT_RECONSTRUCTOR

#define SOT_STATE_OBSERVATION_CHECK_UNIQUENESS_IN_TIME

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

#include <sot/core/matrix-geometry.hh>

#include <state-observation/tools/miscellaneous-algorithms.hpp>

namespace sotStateObservation
{
  /**
     \brief
  */
  class InputReconstructor :
    public dynamicgraph::Entity,
    private boost::noncopyable //
  {
  public:
    /**
       \brief Constructor by name
    */
    InputReconstructor(const std::string& inName);

    ~InputReconstructor();

    /// Each entity should provide the name of the class it belongs to
    virtual const std::string& getClassName (void) const
    {
      return CLASS_NAME;
    }

    /// Header documentation of the python class
    virtual std::string getDocString () const
    {
      return
        "Entity that reconstructs the input signal for the state observation for HRP-2";
    }

    void setFootBias1 (const ::dynamicgraph::Vector & b)
    {
      bias_[0]=b;
    }

    void setFootBias2 (const ::dynamicgraph::Vector & b)
    {
      bias_[1]=b;
    }

    void setFDInertiaDot(const bool& b)
    {
      derivateInertiaFD_=b;
    }

    void setSamplingPeriod(const double& dt)
    {
      dt_=dt;
    }

    void setConfig(const dynamicgraph::Vector& config)
    {
      for(int i=0;i<3;++i)
        {
          if(config(i)==1) config_[i]=true;
          if(config(i)==0) config_[i]=false;
        }
    }

    void setLastInertia(const dynamicgraph::Matrix & inert)
    {
      const dynamicgraph::Matrix& homoWaist=positionWaistSIN.access(currentTime);
      const dynamicgraph::Vector& comVector=comVectorSIN.access(currentTime);
      computeInert(inert,homoWaist,lastInertia_,comVector);
    }

    /**
       \name Parameters
       @{
    */
  protected:
    /*
      \brief Class name
    */
    static const std::string CLASS_NAME;


  private:
    /**
     */

    void computeInert
    (const dynamicgraph::Matrix & inertia, const dynamicgraph::Matrix & homoWaist,
     dynamicgraph::Vector&, const dynamicgraph::Vector&);

    void computeInertDot
    (const dynamicgraph::Matrix & inertia, const dynamicgraph::Vector & dinertia,
     const dynamicgraph::Matrix & homoWaist, dynamicgraph::Vector&, const dynamicgraph::Vector&);

    dynamicgraph::Vector& computeInput
    (dynamicgraph::Vector & input, const int& inTime);

    /**
       \brief Com and derivatives
    */
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> comVectorSIN;

    /**
       \brief Inertia and derivative
    */
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> inertiaSIN;
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> dinertiaSIN;

    /**
       \brief Angular momentum and derivative
    */
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> angMomentumSIN;
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> dangMomentumSIN;
    dynamicgraph::SignalPtr < ::dynamicgraph::Matrix, int> positionWaistSIN;

    /**
       \brief IMU vector
    */
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> imuVectorSIN;

    /**
       \brief contacts position
    */
    dynamicgraph::SignalPtr < unsigned, int> nbContactsSIN;
    dynamicgraph::SignalPtr < ::dynamicgraph::Vector, int> contactsPositionSIN;


    /**
       \brief input
    */
    dynamicgraph::Signal< dynamicgraph::Vector, int> inputSOUT;

    ::dynamicgraph::Vector bias_[2];

    bool derivateInertiaFD_;

    ::dynamicgraph::Vector lastInertia_;

    std::vector<bool> config_;

    int currentTime;
    double dt_;

  };

} // namespace sotStateObservation

#endif //SOT_DYNAMIC_GRAPH_INPUT_RECONSTRUCTOR
