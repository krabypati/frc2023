// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveBase;
import frc.robot.Constants;


public class BalanceBeamCmd extends CommandBase {
  private DriveBase m_DriveBase;

  private double error;
  private double currentAngle;
  private double drivePower;
  
  public BalanceBeamCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
      this.m_DriveBase = RobotContainer.m_driveBase;
      addRequirements(m_DriveBase);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentAngle = m_DriveBase.getPitch();

    error = Constants.beam_final - currentAngle;
    drivePower = -Math.min(Constants.beam_KP * error, 1);

    //Extra push if needed (in case of weight imbalances)
    if (drivePower < 0){
      drivePower *= Math.copySign(0.4, drivePower);
    }
    m_DriveBase.mdrive(drivePower, drivePower, drivePower, drivePower);
    // Debugging prints
    System.out.println("Current Angle: " + currentAngle);
    System.out.println("Error " + error);
    System.out.println("Drive Power: " + drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveBase.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < Constants.beam_threshold; //End when angle approaches threshold (gyro pitch = 0)
  }
}
