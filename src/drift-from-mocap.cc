#include <sot-state-observation/drift-from-mocap.hh>

#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/factory.h>

namespace sotStateObservation
{
  using dynamicgraph::command::makeCommandVoid0;
  using dynamicgraph::command::docCommandVoid0;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( DriftFromMocap, "DriftFromMocap" );

  DriftFromMocap::DriftFromMocap(const std::string& inName):
    Entity(inName),
    limbGlobalSIN(0x0 , "DriftFromMocap("+inName+")::input(MatrixHomogeneous)::limbGlobal"),
    limbLocalSIN(0x0 , "DriftFromMocap("+inName+")::input(MatrixHomogeneous)::limbLocal"),
    driftSOUT( limbGlobalSIN<<limbLocalSIN, "DriftFromMocap("+inName+")::output(MatrixHomogeneous)::drift"),
    driftVectorSOUT( limbGlobalSIN<<limbLocalSIN, "DriftFromMocap("+inName+")::output(Vector)::driftVector"),
    driftInvSOUT( limbGlobalSIN<<limbLocalSIN, "DriftFromMocap("+inName+")::output(MatrixHomogeneous)::driftInv"),
    driftInvVectorSOUT( limbGlobalSIN<<limbLocalSIN, "DriftFromMocap("+inName+")::output(Vector)::driftInvVector")
  {
    signalRegistration (limbGlobalSIN);
    signalRegistration (limbLocalSIN);
    signalRegistration (driftSOUT);
    signalRegistration (driftVectorSOUT);
    signalRegistration (driftInvSOUT);
    signalRegistration (driftInvVectorSOUT);

    driftSOUT.setFunction(boost::bind(&DriftFromMocap::computeDrift,
                                    this, _1, _2));

    driftVectorSOUT.setFunction(boost::bind(&DriftFromMocap::computeDriftVector,
                                    this, _1, _2));

    driftInvSOUT.setFunction(boost::bind(&DriftFromMocap::computeDriftInv,
                                    this, _1, _2));

    driftInvVectorSOUT.setFunction(boost::bind(&DriftFromMocap::computeDriftInvVector,
                                    this, _1, _2));

    std::string docstring;

    addCommand ("init",
                makeCommandVoid0 (*this, &DriftFromMocap::init,
                                  docCommandVoid0 ("reset the drift to zero")));
    init_.setIdentity();
    lastDrift_.setIdentity();

    initialized_ = false;

  }

  DriftFromMocap::~DriftFromMocap()
  {}

  void DriftFromMocap::init()
  {
    int i= driftSOUT.getTime();
    const ::dynamicgraph::sot::MatrixHomogeneous & limbGlobal = limbGlobalSIN(i+1);
    const ::dynamicgraph::sot::MatrixHomogeneous & limbLocal = limbLocalSIN(i+1);

    init_ = limbLocal * limbGlobal.inverse();

    initialized_ = true;
  }

  ::dynamicgraph::sot::MatrixHomogeneous& DriftFromMocap::computeDrift
              (::dynamicgraph::sot::MatrixHomogeneous & drift, const int& inTime)
  {
    if (initialized_)
    {
      const ::dynamicgraph::sot::MatrixHomogeneous & limbGlobal = limbGlobalSIN(inTime);
      const ::dynamicgraph::sot::MatrixHomogeneous & limbLocal = limbLocalSIN(inTime);

      drift = lastDrift_ = init_ * limbGlobal * limbLocal.inverse();
    }
    else
    {
      drift.setIdentity();
      lastDrift_.setIdentity();
    }

    return drift;
  }

  ::dynamicgraph::Vector& DriftFromMocap::computeDriftVector
              (::dynamicgraph::Vector & drift, const int& inTime)
  {
    if(drift.size() !=6)
      drift.resize(6);

    const ::dynamicgraph::sot::MatrixHomogeneous & driftMatrix = driftSOUT(inTime);
    const dynamicgraph::sot::VectorUTheta ut(driftMatrix.linear());
    const Eigen::Vector3d& t = driftMatrix.translation();
    drift.head<3>() = t;
    drift.segment<3>(3) = ut.angle()* ut.axis();

    return drift;
  }

  ::dynamicgraph::sot::MatrixHomogeneous& DriftFromMocap::computeDriftInv
              (::dynamicgraph::sot::MatrixHomogeneous & driftInv, const int& inTime)
  {
    const ::dynamicgraph::sot::MatrixHomogeneous & driftMatrix = driftSOUT(inTime);

    driftInv = driftMatrix.inverse();
    return driftInv;
  }

  ::dynamicgraph::Vector& DriftFromMocap::computeDriftInvVector
              (::dynamicgraph::Vector & driftInv, const int& inTime)
  {
    if(driftInv.size() != 6)
      driftInv.resize(6);

    const ::dynamicgraph::sot::MatrixHomogeneous & driftInvMatrix = driftInvSOUT(inTime);
    const dynamicgraph::sot::VectorUTheta ut(driftInvMatrix.linear());

    driftInv.head<3>() = driftInvMatrix.translation();
    driftInv.segment<3>(3) = ut.angle()*ut.axis();

    return driftInv;
  }



}
