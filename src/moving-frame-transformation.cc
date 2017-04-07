#include <sstream>

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>



#include <dynamic-graph/factory.h>

#include <sot-state-observation/moving-frame-transformation.hh>


namespace sotStateObservation
{
  using dynamicgraph::command::makeCommandVoid0;
  using dynamicgraph::command::docCommandVoid0;
  using dynamicgraph::command::docDirectSetter;
  using dynamicgraph::command::makeDirectSetter;
  using dynamicgraph::command::docDirectGetter;
  using dynamicgraph::command::makeDirectGetter;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN ( MovingFrameTransformation,
                                       "MovingFrameTransformation" );

  MovingFrameTransformation::MovingFrameTransformation
  ( const std::string & inName):
    Entity(inName),
    gMlSIN(0x0 , "MovingFrameTransformation("+inName+")::input(MatrixHomogeneous)::gMl"),
    gVlSIN(0x0 , "MovingFrameTransformation("+inName+")::input(vector)::gVl"),
    gAlSIN(0x0 , "MovingFrameTransformation("+inName+")::input(vector)::gAl"),
    lM0SIN(0x0 , "MovingFrameTransformation("+inName+")::input(MatrixHomogeneous)::lM0"),
    lV0SIN(0x0 , "MovingFrameTransformation("+inName+")::input(vector)::lV0"),
    lA0SIN(0x0 , "MovingFrameTransformation("+inName+")::input(vector)::lA0"),
    lP0SIN(0x0 , "MovingFrameTransformation("+inName+")::input(vector)::lP0"),
    gM0SOUT( gMlSIN<<lM0SIN,
             "MovingFrameTransformation("+inName+")::output(MatrixHomogeneous)::gM0"),
    gV0SOUT( gMlSIN<<gVlSIN<<lM0SIN<<lV0SIN,
             "MovingFrameTransformation("+inName+")::output(vector)::gV0"),
    gA0SOUT( gMlSIN<<gVlSIN<<lM0SIN<<lV0SIN<<lA0SIN<<gAlSIN,
             "MovingFrameTransformation("+inName+")::output(vector)::gA0"),
    gP0SOUT( gMlSIN<<lP0SIN,
             "MovingFrameTransformation("+inName+")::output(vector)::gP0"),
    yawRemoved_(false), pointMode_(false)
  {
    signalRegistration (gMlSIN);
    signalRegistration (gVlSIN);
    signalRegistration (gAlSIN);
    signalRegistration (lM0SIN);
    signalRegistration (lV0SIN);
    signalRegistration (lA0SIN);
    signalRegistration (lP0SIN);
    signalRegistration (gM0SOUT);
    signalRegistration (gV0SOUT);
    signalRegistration (gA0SOUT);
    signalRegistration (gP0SOUT);



    dynamicgraph::Vector velocity (6);
    dynamicgraph::Matrix homoMatrix(4,4);

    homoMatrix.setIdentity();

    gMlSIN.setConstant(homoMatrix);
    gVlSIN.setConstant(velocity);
    lM0SIN.setConstant(homoMatrix);
    lV0SIN.setConstant (velocity);
    gM0SOUT.setConstant(homoMatrix);
    gV0SOUT.setConstant(velocity);


    gM0SOUT.setFunction(boost::bind(&MovingFrameTransformation::computegM0,
                                    this, _1, _2));

    gV0SOUT.setFunction(boost::bind(&MovingFrameTransformation::computegV0,
                                    this, _1, _2));

    gA0SOUT.setFunction(boost::bind(&MovingFrameTransformation::computegA0,
                                    this, _1, _2));

    gP0SOUT.setFunction(boost::bind(&MovingFrameTransformation::computegP0,
                                    this, _1, _2));

    addCommand ("setYawRemoved",
                makeDirectSetter (*this, &yawRemoved_,
                                  docDirectSetter
                                  ("Sets if yes or no, the yaw component gMl anv gVl should be removed",
                                   "bool")));
    addCommand ("getYawRemoved",
                makeDirectGetter (*this, &yawRemoved_,
                                  docDirectGetter
                                  ("Gets if yes or no, the yaw component of gMl anv gVl is be removed",
                                   "bool")));

    addCommand ("setPointMode",
                makeDirectSetter (*this, &pointMode_,
                                  docDirectSetter
                                  ("Sets if the local frame is a point (true) or a homogenous matrix (false)",
                                   "bool")));
    addCommand ("getPointMode",
                makeDirectGetter (*this, &pointMode_,
                                  docDirectGetter
                                  ("Sets if the local frame is a point (true) or a homogenous matrix (false)",
                                   "bool")));

    std::string docstring;
  }

  void removeYaw(::dynamicgraph::sot::MatrixHomogeneous & M)
  {
    dynamicgraph::sot::VectorUTheta v;

    dynamicgraph::sot::MatrixRotation R;
    dynamicgraph::Vector p(3);
    M.extract(p);
    M.extract(R);
    v.fromMatrix(R);
    v(2)=0;
    v.toMatrix(R);

    M.buildFrom(R,p);
  }

  MovingFrameTransformation::~MovingFrameTransformation()
  {
  }

  dynamicgraph::Vector& MovingFrameTransformation::computegV0
  (dynamicgraph::Vector & velocity, const int& inTime)
  {
    dynamicgraph::sot::MatrixHomogeneous gMl (gMlSIN(inTime));
    const dynamicgraph::sot::MatrixHomogeneous & lM0 = lM0SIN(inTime);

    dynamicgraph::Vector  gVl (gVlSIN(inTime));
    const dynamicgraph::Vector &lV0 = lV0SIN(inTime);

    if (yawRemoved_)
    {

        gVl(2) =0;
        removeYaw(gMl);
    }

    dynamicgraph::Matrix gRl(3,3);

    dynamicgraph::Vector lT0(3);


    gMl.extract(gRl);

    lM0.extract(lT0);

    if (pointMode_)
    {
      velocity.resize(3);
    }
    else
    {
      velocity.resize(6);
    }

    const dynamicgraph::Vector& gOmegal = gVl.segment<3>(3);

    velocity.head<3>() = gOmegal.cross(gRl * lT0)
      + gRl * lV0.head<3>()
      + gVl.head<3>();

    if (!pointMode_)
      velocity.segment<3>(3) = gRl * lV0.segment<3>(3) + gOmegal;

    return velocity;

  }


    dynamicgraph::Vector& MovingFrameTransformation::computegA0
  (dynamicgraph::Vector & acceleration, const int& inTime)
  {
    dynamicgraph::sot::MatrixHomogeneous gMl(gMlSIN(inTime));
    const dynamicgraph::sot::MatrixHomogeneous & lM0 = lM0SIN(inTime);

    dynamicgraph::Vector gVl(gVlSIN(inTime));
    const dynamicgraph::Vector &lV0 = lV0SIN(inTime);

    dynamicgraph::Vector gAl(gAlSIN(inTime));
    const dynamicgraph::Vector &lA0 = lA0SIN(inTime);

    if (yawRemoved_)
    {
        gAl(2) = 0;
        gVl(2) = 0;
        removeYaw(gMl);
    }

    dynamicgraph::Matrix gRl(3,3);

    dynamicgraph::Vector lT0(3);

    gMl.extract(gRl);

    lM0.extract(lT0);

    if (pointMode_)
    {
      acceleration.resize(3);
    }
    else
    {
      acceleration.resize(6);
    }


    const Eigen::Vector3d& gOmegal = gVl.segment<3>(3);

    const Eigen::Vector3d& gOmegaDotl = gAl.segment<3>(3);


    acceleration.head<3>() =  gOmegaDotl.cross(gRl * lT0)
      + gOmegal.cross(gOmegal.cross(gRl * lT0))
      + 2*gOmegal.cross(gRl * lV0.head<3>())
      + gRl * lA0.head<3>()
      + gAl.head<3>();

    if (!pointMode_)
      acceleration.segment<3>(3) = gOmegal.cross(gRl * lV0.segment<3>(3))
        + gRl * lA0.segment<3>(3) + gOmegaDotl;

    return acceleration;

  }

  ::dynamicgraph::sot::MatrixHomogeneous& MovingFrameTransformation::computegM0
  (::dynamicgraph::sot::MatrixHomogeneous & homo, const int& inTime)
  {
    dynamicgraph::sot::MatrixHomogeneous gMl(gMlSIN(inTime));
    const dynamicgraph::sot::MatrixHomogeneous & lM0 = lM0SIN(inTime);


    if (yawRemoved_)
    {
        removeYaw(gMl);
    }

    homo = gMl * lM0;

    //std::cout << "computegM0" << std::endl;

    return homo;
  }

    dynamicgraph::Vector& MovingFrameTransformation::computegP0
  (dynamicgraph::Vector & position, const int& inTime)
  {

    if (pointMode_)
    {
      dynamicgraph::sot::MatrixHomogeneous gMl(gMlSIN(inTime));
      const dynamicgraph::Vector & lP0 = lP0SIN(inTime);


      if (yawRemoved_)
      {
        removeYaw(gMl);
      }

      gMl.multiply(lP0,position);
    }
    else
    {
      dynamicgraph::sot::MatrixHomogeneous gM0(gM0SOUT(inTime));
      ::dynamicgraph::sot::MatrixRotation R;
      ::dynamicgraph::sot::VectorUTheta ut;
      ::dynamicgraph::Vector t(3);
      gM0.extract(R);
      gM0.extract(t);
      ut.fromMatrix(R);
      position.resize(6);
      position.head<3> = t;
      position.tail<3> = ut;
    }

    return position;
  }

}
