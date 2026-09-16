// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //Meters
  public static final double NEAR_DISTANCE = 0;
  public static final double MID_DISTANCE  = 0;
  public static final double FAR_DISTANCE  = 0;
  //Motor rotations
  public static final double NEAR_HOOD_POS = 0;
  public static final double MID_HOOD_POS  = 0;
  public static final double FAR_HOOD_POS  = 0;

  public static final double NEAR_SHOOTER_VELOCITY = 0;
  public static final double MID_SHOOTER_VELOCITY  = 0;
  public static final double FAR_SHOOTER_VELOCITY  = 0;

  public static class OperatorConstants
  {
    //
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;

    // speed multiplier :D
    public static final double SPEED_CONTROL = 0.5;
  }

  public static class SwerveConstants {
    public static final double kPX = 2.75;
    public static final double kPY = 2.75;
    public static final double kPTheta = 2.5;
    public static final double kXTolerance = 0;
    public static final double kYTolerance = 0;
    public static final double kThetaTolerance = 0;
    public static double kStoredRadius = 3.9527559/2; // to be configured later??
    public static double kDrivebaseRadius = .409;
  }

  public static class LauncherConstants{

    // For Launcher - Do we need KG? michaudc 05 Sep 26
    // Numbers from OffseasonV2 Original Settings
    private static final Slot0Configs slot0Configs = new Slot0Configs()
    .withKS(.1)
    .withKV(.12)
    .withKP(11)
    .withKI(0)
    .withKD(0);

  public static final TalonFXConfiguration launcherConfig = new TalonFXConfiguration()
    .withSlot0(slot0Configs);

    // Preset Velocities - will need to adjust for V2 Robot - michaudc 05 Sep
    public static final double NEAR_SHOOTER_VELOCITY = 40;
    public static final double MID_SHOOTER_VELOCITY = 47;
    public static final double FAR_SHOOTER_VELOCITY = 75;
    public static final double FIRST_MID_SHOT = 65;
    public static final double STOP_LAUNCHER = 0;

    public static final int leftIndexerID = 32;
    public static final int rightIndexerID = 33;
    public static final int leftShooterID = 30;
    public static final int rightShooterID = 31;
    public static final int hoodMotorID = 34;
    public static final int activeFloorFrontID = 40;
    public static final int activeFloorBackID = 41;
    public static final int launcherCurrentLimit = 60;

    
  }

  public static class IntakeConstants{
    public static final int intakeID = 51;
    public static final int intakeIDLeft = 50;
    public static final int pivotMotorID = 52;
    public static final double ROLLER_SPEED = -0.7;
    public static final double REVERSE_ROLLER_SPEED = 0.7;
    public static final double STOP_INTAKE = 0;
    public static final double PIVOT_SPEED = .2;
  

    private static final Slot0Configs kSlot0Configs = new Slot0Configs()
    .withKA(0)
    .withKG(.3)
    .withKS(.0)
    .withKV(0)
    .withKP(0)
    .withKI(0)
    .withKD(0);

    public static TalonFXConfiguration intakeConfig = new TalonFXConfiguration().withSlot0(kSlot0Configs);

    // Config Setup for Pivot Motor - michaudc 05 Sep 26
    private static final Slot0Configs kSlot0ConfigsPivot = new Slot0Configs()
    .withKA(0)
    .withKG(.3)
    .withKS(.0)
    .withKV(0)
    .withKP(0.35) // Values from 6340 Alpha
    .withKI(0)
    .withKD(0);

    public static TalonFXConfiguration pivotConfig = new TalonFXConfiguration().withSlot0(kSlot0ConfigsPivot);
  }

  // Constants for Swerve Drive
  public static final double MAX_SPEED = Units.feetToMeters(14.5);

  // Not Sure if we need these
  public static final double leftAlignmentX = .2435737274077523; //meters
    public static final double leftAlignmentY = 0.275;
    public static final double rightAlignmentX = .2435737274077523;
    public static final double rightAlignmentY = 0.623;
    public static final double troughAlignmentTheta = -1.860;
    public static final double troughAlignmentX = .379;
    public static final double troughAlignmentY = .748;
    public static final double thetaAlignment = -Math.PI/2; //degrees
    public static double maxAlignmentDistance = 1.5;

    public static final double xTolerance = .05;
    public static final double yTolerance = .05;
    public static final double thetaTolerance = .05;
}
