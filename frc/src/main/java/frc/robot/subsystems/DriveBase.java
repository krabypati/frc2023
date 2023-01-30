// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

//mecanum imports
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import frc.robot.Constants.goofyTalons;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.controller;
import frc.robot.Constants.goofySpeed;




public class DriveBase extends SubsystemBase {
  private AHRS navx = new AHRS(SerialPort.Port.kUSB);

  public static WPI_TalonSRX m_leftFront = new WPI_TalonSRX(goofyTalons.leftFrontID);
  public static WPI_TalonSRX m_rightFront = new WPI_TalonSRX(goofyTalons.rightFrontID);
  public static WPI_TalonSRX m_leftFollow = new WPI_TalonSRX(goofyTalons.leftFollowID);
  public static WPI_TalonSRX m_rightFollow = new WPI_TalonSRX(goofyTalons.rightFollowID);

  public static MecanumDrive meca;


  public DriveBase() {
    final MotorControllerGroup frontLeft = new MotorControllerGroup(m_leftFront);
    frontLeft.setInverted(false);
    final MotorControllerGroup frontRight = new MotorControllerGroup(m_rightFront); //final MotorControllerGroup frontRight = new MotorControllerGroup(rightFront, rightBack);
    frontRight.setInverted(true);
    final MotorControllerGroup backLeft = new MotorControllerGroup(m_leftFollow);
    frontLeft.setInverted(false);
    final MotorControllerGroup backRight = new MotorControllerGroup(m_rightFollow);
    frontRight.setInverted(true);

    meca = new MecanumDrive(frontLeft, backLeft, frontRight, backRight);
  }
  public void mdrive(double lFS, double rFS, double lfS, double rfS){
    // lFS = left Front speed, lfs = left follow speed etc
    m_rightFront.set(rFS);
    m_leftFront.set(lFS);
    m_rightFollow.set(rfS);
    m_leftFollow.set(lfS);
  }
  public void stop(){
    mdrive(0, 0, 0, 0);
  }

  //Navx goofy methods
  public void goofyGyro(){
    navx.calibrate();
  }
  public void zeroGyro(){
    navx.reset();
  }
  public double getYaw(){
    return navx.getYaw();
  }
  public double getPitch(){
    return navx.getPitch();
  }
  public double getRoll(){
    return navx.getRoll();
  }
  public double getAngle(){
    return navx.getAngle();
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // This method will be called once per scheduler run
    /*double l2 = RobotContainer.controller.getRawButton(L2)? 0.8: 0.4;
        drive(l2 * maxSpeed * RobotContainer.controller.getRawAxis(1),
            l2 * maxSpeed * RobotContainer.controller.getRawAxis(1),
            l2 * maxRotation * RobotContainer.controller.getRawAxis(1)/RobotContainer.controller.getRawAxis(0) );*/
    
    double l2 = RobotContainer.controller.getRawButton(controller.L2)? 0.8 : 0.4;
    double lfV =  Math.sin( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double rfV =  Math.cos( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double lbV =  Math.cos( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double rbV =  Math.sin( Math.atan(RobotContainer.controller.getRawAxis(1)/ RobotContainer.controller.getRawAxis(0) + Math.PI /4));
    double vrot =  Math.sqrt(Math.pow(RobotContainer.controller.getRawAxis(1), 2) + Math.pow(RobotContainer.controller.getRawAxis(0), 2));
    //lf = v sin(theta + pi/4) + vrot
    //rf = v cos(theta + pi/4) - vrot
    //lb = v cos(theta + pi/4) + vrot
    //rb = v sin(theta + pi/4) - vrot
    //Drive Straight  
    if (RobotContainer.controller.getRawAxis(1) < -0.3 || RobotContainer.controller.getRawAxis(1) > 0.3){
      mdrive(l2 * -RobotContainer.controller.getRawAxis(1), 
      l2 * -RobotContainer.controller.getRawAxis(1),
      l2* -RobotContainer.controller.getRawAxis(1), 
      l2* -RobotContainer.controller.getRawAxis(1));
  }

  //Hori Straight
  else if (RobotContainer.controller.getRawAxis(0) < -0.3 || RobotContainer.controller.getRawAxis(0) > 0.3){ 
      mdrive(l2 * RobotContainer.controller.getRawAxis(0) ,
      l2 * -RobotContainer.controller.getRawAxis(0), 
      l2 * - RobotContainer.controller.getRawAxis(0), 
      l2 * RobotContainer.controller.getRawAxis(0));
  }
  
  //Cross  
  else if ((RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) < -0.3) || 
          (RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) > 0.3) ||
          (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) > 0.3) ||
          (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) < -0.3)) {
      mdrive(goofySpeed.maxSpeed * lfV + vrot, goofySpeed.maxSpeed * rfV - vrot, goofySpeed.maxSpeed * lbV + vrot, goofySpeed.maxSpeed * rbV - vrot);
  }
/*45o
  //Cross Forward Right 
  else if (RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) > 0.3){
      mdrive(0, l2 * goofySpeed.maxSpeed , l2 * goofySpeed.maxSpeed, 0);
  }
  //Cross Forward Left 
  else if (RobotContainer.controller.getRawAxis(1) < -0.3 && RobotContainer.controller.getRawAxis(0) < -0.3){
      mdrive(l2 * goofySpeed.maxSpeed, 0 ,0 ,l2 * goofySpeed.maxSpeed);
  }
  //Cross Backward Right
  else if (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) > 0.3){
      mdrive(l2 * -goofySpeed.maxSpeed , 0 ,l2 * -goofySpeed.maxSpeed ,0);
  }
  //Cross Forward Left 
  else if (RobotContainer.controller.getRawAxis(1) > 0.3 && RobotContainer.controller.getRawAxis(0) < -0.3){
      mdrive(0, l2 * -goofySpeed.maxSpeed,0 ,l2 * -goofySpeed.maxSpeed);
  }*/

  //Turn right
  if (RobotContainer.controller.getRawAxis(2) > 0.3){
      mdrive(l2 * goofySpeed.maxSpeed, l2 * -goofySpeed.maxSpeed, l2 * goofySpeed.maxSpeed, l2 * -goofySpeed.maxSpeed);
  }
  //Turn left
  else if (RobotContainer.controller.getRawAxis(2) > 0.3){
      mdrive(- l2 * goofySpeed.maxSpeed, l2 * goofySpeed.maxSpeed, l2 * -goofySpeed.maxSpeed, l2 * goofySpeed.maxSpeed);
  }
  //Curve traj right
  else if (RobotContainer.controller.getRawAxis(2) > 0.5 && RobotContainer.controller.getRawAxis(5) < -0.5){
      mdrive(l2 * goofySpeed.maxSpeed, l2 * goofySpeed.minSpeed, l2 * goofySpeed.maxSpeed, l2 * goofySpeed.minSpeed);
  }
  //Curve traj left
  else if (RobotContainer.controller.getRawAxis(2) > 0.5 && RobotContainer.controller.getRawAxis(5) < -0.5){
      mdrive(l2 * goofySpeed.minSpeed, l2 * goofySpeed.maxSpeed, l2 * goofySpeed.minSpeed, l2 * goofySpeed.maxSpeed);
  }
  }
}
