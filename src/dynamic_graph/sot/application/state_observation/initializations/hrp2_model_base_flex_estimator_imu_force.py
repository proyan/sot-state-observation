# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 
import sys
import numpy as np
from numpy import *
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb

from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose
from dynamic_graph.sot.application.state_observation import DGIMUModelBaseFlexEstimation, PositionStateReconstructor, InputReconstructor, Odometry, DriftFromMocap

from dynamic_graph.sot.core.derivator import Derivator_of_Vector

from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.application.state_observation import Calibrate


class HRP2ModelBaseFlexEstimatorIMUForce(DGIMUModelBaseFlexEstimation):
    def __init__(self, robot, name='flextimator2'):

        DGIMUModelBaseFlexEstimation.__init__(self,name)
        self.setSamplingPeriod(0.005)  
        self.robot = robot
	self.setContactModel(1)
        self.robot.dynamic.inertia.recompute(1)					      
        self.robot.dynamic.waist.recompute(1)	
        self.robot.frames['leftFootForceSensor'].position.recompute(1)
	self.robot.frames['rightFootForceSensor'].position.recompute(1)

	self.setWithForceSensors(True)
	self.setForceVariance(1e-4)
	self.setWithComBias(False)
        
	self.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6+(1.e-13,)*2+(1e-8,)*3)))
	self.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3)))

	# Odometry
        self.odometry=Odometry ('Odometry')
	plug (self.robot.device.state,self.odometry.robotStateIn)
#	self.odometry.setLeftFootPosition(self.robot.frames['leftFootForceSensor'].position.value)
#	self.odometry.setRightFootPosition(self.robot.frames['rightFootForceSensor'].position.value)
	plug(self.robot.frames['leftFootForceSensor'].position,self.odometry.leftFootPositionRef)
	plug(self.robot.frames['rightFootForceSensor'].position,self.odometry.rightFootPositionRef)
	plug (self.robot.device.forceLLEG,self.odometry.force_lf)
        plug (self.robot.device.forceRLEG,self.odometry.force_rf)
        plug (self.robot.frames['rightFootForceSensor'].position,self.odometry.rightFootPosition)
        plug (self.robot.frames['leftFootForceSensor'].position,self.odometry.leftFootPosition)
	plug (self.odometry.nbSupport,self.contactNbr)

	# Contacts definition
	self.contacts = Stack_of_vector ('contacts')
        plug(self.odometry.supportPos1,self.contacts.sin1)
        plug(self.odometry.supportPos2,self.contacts.sin2)
	self.contacts.selec1 (0, 6)
	self.contacts.selec2 (0, 6)

        # Sensors stack

		# IMU
        self.sensorStackimu = Stack_of_vector (name+'SensorsIMU')
        plug(self.robot.device.accelerometer,self.sensorStackimu.sin1)
        plug(self.robot.device.gyrometer,self.sensorStackimu.sin2)
        self.sensorStackimu.selec1 (0, 3)
        self.sensorStackimu.selec2 (0, 3)


                # Drift
        self.drift = DriftFromMocap(name+'Drift')
        

	# Optional measurenents Stack (forces + drift)
        self.sensorStackOptional = Stack_of_vector (name+'SensorsOPT')
        self.sensorStackOptional.selec1toEnd (0)
        self.sensorStackOptional.selec2toEnd (0)
        plug(self.odometry.forceSupportStack,self.sensorStackOptional.sin1)
        plug(self.drift.driftVector,self.sensorStackOptional.sin2)
        
        self.contactForces = self.odometry.forceSupportStack

		# Calibration
	self.calibration= Calibrate('calibration')
	plug(self.odometry.nbSupport,self.calibration.contactsNbr)
	plug(self.robot.dynamic.com,self.calibration.comIn)
	plug(self.contacts.sout,self.calibration.contactsPositionIn)
	plug(self.sensorStackimu.sout,self.calibration.imuIn)	

		# Concatenating
        self.sensorStack = Stack_of_vector (name+'Sensors')
        plug(self.sensorStackimu.sout,self.sensorStack.sin1)
        plug(self.sensorStackOptional.sout,self.sensorStack.sin2)
        self.sensorStack.selec1 (0, 6)
        self.sensorStack.selec2 (0, 12)
	plug(self.sensorStack.sout,self.measurement);
     
        # Input reconstruction

		# IMU Vector
        self.inputPos = MatrixHomoToPoseUTheta(name+'InputPosition')
        plug(robot.frames['accelerometer'].position,self.inputPos.sin)
        self.robot.dynamic.createJacobian(name+'ChestJ_OpPoint','chest')	
        self.imuOpPoint = OpPointModifier(name+'IMU_oppoint')
        self.imuOpPoint.setEndEffector(False)
        self.imuOpPoint.setTransformation(matrixToTuple(np.linalg.inv(np.matrix(self.robot.dynamic.chest.value))*np.matrix(self.robot.frames['accelerometer'].position.value)))
        plug (self.robot.dynamic.chest,self.imuOpPoint.positionIN)			
        plug (self.robot.dynamic.signal(name+'ChestJ_OpPoint'),self.imuOpPoint.jacobianIN)
        self.inputVel = Multiply_matrix_vector(name+'InputVelocity')
        plug(self.imuOpPoint.jacobian,self.inputVel.sin1)
        plug(self.robot.device.velocity,self.inputVel.sin2)
        self.inputPosVel = Stack_of_vector (name+'InputPosVel')
        plug(self.inputPos.sout,self.inputPosVel.sin1)
        plug(self.inputVel.sout,self.inputPosVel.sin2)
        self.inputPosVel.selec1 (0, 6)
        self.inputPosVel.selec2 (0, 6)
        self.IMUVector = PositionStateReconstructor (name+'EstimatorInput')
        plug(self.inputPosVel.sout,self.IMUVector.sin)
        self.IMUVector.inputFormat.value  = '001111'
        self.IMUVector.outputFormat.value = '011111'
        self.IMUVector.setFiniteDifferencesInterval(2)

        	# CoM and derivatives
        self.com=self.robot.dynamic.com
        self.DCom = Multiply_matrix_vector(name+'DCom')
        plug(self.robot.dynamic.Jcom,self.DCom.sin1)
        plug(self.robot.device.velocity,self.DCom.sin2)
        self.comVectorIn = Stack_of_vector (name+'ComVectorIn')
        plug(self.com,self.comVectorIn.sin1)
        plug(self.DCom.sout,self.comVectorIn.sin2)
        self.comVectorIn.selec1 (0, 3)
        self.comVectorIn.selec2 (0, 3)
        self.comVector = PositionStateReconstructor (name+'ComVector')
        plug(self.comVectorIn.sout,self.comVector.sin)
        self.comVector.inputFormat.value  = '000101'
        self.comVector.outputFormat.value = '010101'  
	self.comVector.setFiniteDifferencesInterval(20)

		# Compute derivative of Angular Momentum
        self.angMomDerivator = Derivator_of_Vector('angMomDerivator')
        plug(self.robot.dynamic.angularmomentum,self.angMomDerivator.sin)
        self.angMomDerivator.dt.value = self.robot.timeStep          
        
        	# Concatenate with InputReconstructor entity
        self.inputVector=InputReconstructor(name+'inputVector')
        plug(self.comVector.sout,self.inputVector.comVector)
        plug(self.robot.dynamic.inertia,self.inputVector.inertia)
        self.inputVector.dinertia.value=(0,0,0,0,0,0)
	plug(self.robot.dynamic.angularmomentum,self.inputVector.angMomentum)
	plug(self.angMomDerivator.sout,self.inputVector.dangMomentum)
        plug(self.robot.dynamic.waist,self.inputVector.positionWaist)
        plug(self.IMUVector.sout,self.inputVector.imuVector)
        plug(self.odometry.nbSupport,self.inputVector.nbContacts)
	plug(self.contacts.sout,self.inputVector.contactsPosition)

        self.inputVector.setSamplingPeriod(robot.timeStep)
        self.inputVector.setFDInertiaDot(True)     
        plug(self.inputVector.input,self.input)
        self.robot.flextimator = self

        self.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
        self.setKfv(matrixToTuple(np.diag((600,600,600))))
        self.setKte(matrixToTuple(np.diag((600,600,600))))
        self.setKtv(matrixToTuple(np.diag((60,60,60))))

