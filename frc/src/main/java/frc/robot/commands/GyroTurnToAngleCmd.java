// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;

public class GyroTurnToAngleCmd extends CommandBase {
  /** Creates a new GyroTurnToAngleCmd. */
  private DriveBase m_DriveBase;
  double angleToTurn;
  double error; //basically difference idk
  double targetAngle;
  public GyroTurnToAngleCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveBase = RobotContainer.m_driveBase;
    addRequirements(m_DriveBase);
    this.angleToTurn = angleToTurn;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetAngle = angleToTurn + m_DriveBase.getAngle();
    System.out.println("Current angle: " + m_DriveBase.getAngle());
    System.out.println("Target angle: " + targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetAngle - m_DriveBase.getAngle();

    double value = error *Constants.gyroTurn_KP;
    if (Math.abs(value) >0.75){
      value = Math.copySign(0.75, value);
    }
    if (Math.abs(value) < 0.15){
      value = Math.copySign(0.15, value);
    }
    //Debug
    System.out.println("error:" + error);
    System.out.println("value:" + value);

    m_DriveBase.mdrive(value, -value, value, -value); //needs fixing bcs this is based on normal tank drive, not mecanum T_T
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.drive_threshold;
  }
}
