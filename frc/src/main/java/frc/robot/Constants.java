// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //Controller
  public static class controller{
    public static final int Sq = 1;
    public static final int X = 2;
    public static final int O = 3;
    public static final int Tri = 4;

    public static final int L1 = 5;
    public static final int R1 = 6;
    public static final int L2 = 7;
    public static final int R2 = 8;

    public static final int L3 = 11;
    public static final int R3 = 12;

    public static final int Share = 9;
    public static final int Option = 10;
    public static final int TouchPad = 13;
    
    public static final int Up = 14;
    public static final int Down = 15;
    public static final int Left = 16;
    public static final int Right = 17;
}

  //Talon IDs
  public static class goofyTalons{
    public static final int leftFrontID = 1;
    public static final int leftFollowID = 2;
    public static final int rightFrontID = 3;
    public static final int rightFollowID = 4;
  }

  public static class goofySpeed{
    public static final double maxSpeed = 0.8;
    public static final double maxRotation = 0.5;
    public static final double minSpeed = 0.4;
  }

  //goofy drivebase consts
  public static final double beam_final = 0;
  public static final double beam_threshold = 1;
  public static final double balancing_mul = 1.35; //needs testing
  public static final double beam_KP = 0.015; //kP (in PID)
  public static final double gyroTurn_KP = 0.007; //kP (in PID)
  public static final double drive_threshold = 3; //turning to an angle threshold

  public static final double AprilTagCap = 0.75;


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
