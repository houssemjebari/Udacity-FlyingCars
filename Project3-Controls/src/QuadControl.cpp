#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"
#include "Math/Angles.h"
#include <cmath>

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]
  float cBar = collThrustCmd / 4.f;
  float pBar = momentCmd.x / (L * sqrt(2) * 4.f);
  float qBar = momentCmd.y / (L * sqrt(2) * 4.f);
  float rBar = momentCmd.z / (kappa * 4.f);
  cmd.desiredThrustsN[0] = CONSTRAIN(cBar + pBar + qBar + rBar,minMotorThrust,maxMotorThrust); // rear right
  cmd.desiredThrustsN[1] = CONSTRAIN(cBar - pBar + qBar - rBar,minMotorThrust,maxMotorThrust); // front left
  cmd.desiredThrustsN[2] = CONSTRAIN(cBar + pBar - qBar - rBar,minMotorThrust,maxMotorThrust);// front right
  cmd.desiredThrustsN[3] = CONSTRAIN(cBar - pBar - qBar + rBar,minMotorThrust,maxMotorThrust); // rear left
  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes
  V3F PQR_DOT = kpPQR * (pqrCmd - pqr);
  V3F inertia(Ixx, Iyy, Izz);
  V3F momentCmd = PQR_DOT * inertia;
  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)
  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  if (collThrustCmd > 0)
  {
      float c = collThrustCmd / mass;
      float bxc = accelCmd[0] / c;
      float byc = accelCmd[1] / c;
      float bxc_dot = kpBank * (bxc - R(0, 2));
      float byc_dot = kpBank * (byc - R(1, 2));
      float p_c = (bxc_dot * R(1, 0) - byc_dot * R(0, 0)) / R(2, 2);
      float q_c = (bxc_dot * R(1, 1) - byc_dot * R(0, 1)) / R(2, 2);
      pqrCmd[0] = p_c;
      pqrCmd[1] = q_c;
  }
  else 
  {
      pqrCmd[0] = 0;
      pqrCmd[1] = 0;
  }
  pqrCmd[2] = 0;  
  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]
  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;
  integratedAltitudeError += KiPosZ * (posZCmd - posZ) * dt;
  float command = integratedAltitudeError + kpPosZ * (posZCmd - posZ) + kpVelZ * (velZCmd - velZ)  + accelZCmd;
  command = CONSTRAIN(command,-maxDescentRate/dt,maxAscentRate/dt);
  thrust = (-command + 9.81f) * mass / R(2, 2);
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;
  V3F accelCmd = accelCmdFF;
  velCmd[0] = min(maxSpeedXY, velCmd[0]);
  velCmd[1] = min(maxSpeedXY, velCmd[1]);
  accelCmd += kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel);
  accelCmd[0] = min(maxAccelXY, accelCmd[0]);
  accelCmd[1] = min(maxAccelXY, accelCmd[1]);
  accelCmd[2] = 0;
  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  float yawRateCmd=0;
  yawCmd = AngleNormF(yawCmd);
  yaw = AngleNormF(yaw);
  float error = (yawCmd - yaw);
  error = AngleNormF(error);
  yawRateCmd = kpYaw * error;
  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
