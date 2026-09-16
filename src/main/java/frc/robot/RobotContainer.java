// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoAimCommand

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  
  private LauncherSubsystem launcherSubsystem = new LauncherSubsystem();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private  Pose2d startPose;
  // Driver Controller init
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // DEBUG 
  final CommandXboxController operatorXbox = new CommandXboxController(1);

  // Operator Controller init
  // final CommandPS4Controller operatorPS4 = new CommandPS4Controller(1);

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));

  private double rotationSpeed = 0.85;

  //
   // Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   //
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -1,
                    () -> driverXbox.getLeftX() * -1) 
                    .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                    .deadband(OperatorConstants.DEADBAND)
                    .scaleTranslation(0.8) // change later
                    .scaleRotation(0.8)
                    .allianceRelativeControl(true); // in Alpha_2025, they make this false & flipDirection based on alliance instead 

  SwerveInputStream driveAngularSlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                    () -> driverXbox.getLeftY() * -.6, // Set these values for slower: 1/2 Speed
                    () -> driverXbox.getLeftX() * -.6) // Set these values for slower: 1/2 Speed
                    .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                    .deadband(OperatorConstants.DEADBAND)
                    .scaleTranslation(0.3)
                    .scaleRotation(0.4)
                    .allianceRelativeControl(true);

  //
  // Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
  //
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  //
  // Clone's the angular velocity input stream and converts it to a robotRelative input stream.
  //
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .scaleRotation(rotationSpeed)
                                                                    .allianceRelativeControl(true);
  
  SwerveInputStream driveAngularVelocityKeyboardSlow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.3)
                                                                    .allianceRelativeControl(true);
                                                                    
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                  .withControllerHeadingAxis(() ->
                                                  Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                                                  () ->
                                                  Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI *2))
                                                  .headingWhile(true)
                                                  .translationHeadingOffset(true)
                                                  .translationHeadingOffset(Rotation2d.fromDegrees(0));

  
  // use SmartDashboard for a list of auto options
  SendableChooser<Command> autoChooser;                                                                         

  

  // The container for the robot. Contains subsystems, OI devices, and commands. 
  
  public RobotContainer() {
    
    
    //Named Commands
    NamedCommands.registerCommand("Start Launcher Near", launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.NEAR_SHOOTER_VELOCITY));
    NamedCommands.registerCommand("Start Launcher Mid", launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.MID_SHOOTER_VELOCITY));
    NamedCommands.registerCommand("Start Launcher Far", launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.FAR_SHOOTER_VELOCITY));
    NamedCommands.registerCommand("Start First Mid Launch", launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.FIRST_MID_SHOT));
    NamedCommands.registerCommand("Stop Launcher",launcherSubsystem.stopShooterCommand());


    NamedCommands.registerCommand("Start Intake", intakeSubsystem.setRollerSpeedCommand(Constants.IntakeConstants.ROLLER_SPEED));
    NamedCommands.registerCommand("Stop Intake", intakeSubsystem.setRollerSpeedCommand(Constants.IntakeConstants.STOP_INTAKE));

    NamedCommands.registerCommand("Start Indexer", launcherSubsystem.startIndexerAndFloorCommand());
    NamedCommands.registerCommand("Stop Indexer", launcherSubsystem.stopIndexerAndFloorCommand());

    NamedCommands.registerCommand("Auto Aim", 
        new AutoAimCommand(
            drivebase, 
            launcherSubsystem, 
            () -> 0.0, 
            () -> 0.0, 
            () -> 0.0
        )
    );


    //launcherSubsystem = new LauncherSubsystem();
    // add auto options to SmartDashboard
    //autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Auto Chooser", autoChooser);

    //intakeSubsystem = new IntakeSubsystem();

    autoChooser = AutoBuilder.buildAutoChooser();
   // boolean isCompetition = true;

    // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    // (stream) -> isCompetition
    // ? stream.filterz

    // )

    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
 

    // Establish Command for different types of Driving
    //Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    //Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity); // Fast Mode
    
    // # ---------------------- Driver Commands ------------------------ #
    // Slow Mode Fix Temporary - Right Trigger Makes the Robot Drive Slow - michaudc
    driverXbox.leftTrigger().whileTrue(drivebase.driveFieldOriented(driveAngularSlow)); //the func one with the supplier chassis speeds

    // reverse intake - VC 6/24/26
    //driverXbox.leftBumper().whileTrue(intakeSubsystem.setRollerSpeedCommand(Constants.IntakeConstants.REVERSE_ROLLER_SPEED));

    //driverXbox.leftBumper().whileTrue(launcherSubsystem.startStopFloorCommand());

    // Zero the Gyro (Reset Field Centric)
    driverXbox.a().onTrue((Commands.runOnce(()->drivebase.zeroGyro())));

    // Default Driving Command
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveAngularVelocity)); // Fast Mode
    //drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveRobotOriented)); //robot centric

    // #--------------------------Operator Commands------------------------#

    operatorXbox.rightBumper().whileTrue(launcherSubsystem.startIndexerAndFloorCommand())
    .whileFalse(launcherSubsystem.stopIndexerAndFloorCommand()
    );

    //operatorXbox.leftBumper().whileTrue(intakeSubsystem.setRollerSpeedCommand(Constants.IntakeConstants.ROLLER_SPEED));

    operatorXbox.leftBumper().whileTrue(launcherSubsystem.reverseIndexerAndFloorCommand())
    .whileFalse(launcherSubsystem.stopIndexerAndFloorCommand()
    );



    //driverXbox.rightBumper().onTrue(launcherSubsystem.setShooterSpeedCmd(rotationSpeed));
    //driverXbox.rightBumper().onTrue(launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.NEAR_SHOOTER_VELOCITY));//70 is nearish
    
    //driverXbox.y().onTrue(launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.MID_SHOOTER_VELOCITY));
    
    //driverXbox.x().onTrue(launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.FAR_SHOOTER_VELOCITY));
    
    //driverXbox.leftBumper().onTrue(launcherSubsystem.stopShooterCommand());
    
    operatorXbox.rightTrigger().whileTrue(intakeSubsystem.setRollerSpeedCommand(Constants.IntakeConstants.ROLLER_SPEED));
    
    operatorXbox.leftTrigger().whileTrue(intakeSubsystem.setRollerSpeedCommand(Constants.IntakeConstants.REVERSE_ROLLER_SPEED));

    operatorXbox.y().onTrue(launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.FAR_SHOOTER_VELOCITY));
                                                       
    operatorXbox.b().onTrue(launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.MID_SHOOTER_VELOCITY));

    operatorXbox.a().onTrue(launcherSubsystem.stopShooterCommand());

    operatorXbox.x().onTrue(launcherSubsystem.setShooterVelocityCommand(Constants.LauncherConstants.NEAR_SHOOTER_VELOCITY));
    //practice controls


    // Testing Intake 
    driverXbox.rightBumper().onTrue(intakeSubsystem.intakeUpCommand());
    driverXbox.leftBumper().onTrue(intakeSubsystem.intakeDownCommand());

    driverXbox.rightTrigger().whileTrue(intakeSubsystem.setRollerSpeedCommand(Constants.IntakeConstants.ROLLER_SPEED));

    // These are the Testing Methods using velocity
    //driverXbox.povUp().onTrue(launcherSubsystem.hoodUpCommand());
    //driverXbox.povDown().onTrue(launcherSubsystem.hoodDownCommand());
    //driverXbox.povRight().onTrue(launcherSubsystem.hoodStopCommand());

    // Hood Methods Using Motion Magic - michaudc
    driverXbox.povUp().onTrue(launcherSubsystem.setHoodPositionCommand(60)); // Max Deploy
    driverXbox.povRight().onTrue(launcherSubsystem.setHoodPositionCommand(35)); // Mid Deploy
    driverXbox.povDown().onTrue(launcherSubsystem.setHoodPositionCommand(2)); // All the way down

  driverXbox.x().whileTrue(
      new AutoAimCommand(
          drivebase,
          launcherSubsystem,
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX(),
          () -> -driverXbox.getRightX()
      )
  );

    // Pivot Speed for Test
    //driverXbox.y().whileTrue(intakeSubsystem.setIntakePivotSpeedCommand(Constants.IntakeConstants.PIVOT_SPEED));
    //driverXbox.b().whileTrue(intakeSubsystem.setIntakePivotSpeedCommand(-Constants.IntakeConstants.PIVOT_SPEED));
    


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    
    Command autoCommand = autoChooser.getSelected();
    PathPlannerAuto auto = (PathPlannerAuto) autoCommand;

    Pose2d startPose = auto.getStartingPose();
    drivebase.setStartPose(startPose);

    return autoChooser.getSelected();
    
    

  }
}
