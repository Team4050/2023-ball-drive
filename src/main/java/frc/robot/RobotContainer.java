// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Operator;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.ClawToggleCmd;
import frc.robot.commands.DriveToPosition;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BallDriveModule;
import frc.robot.subsystems.BallDriveSubsystem;
import frc.robot.subsystems.BallDrivetrainConfig;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 /**
  * Scale input joystick rotational velocity - done
  * Normalize to maximum module velocity - done
  * Rate limit acceleration per module - done
  * Balance older motors?
  * Check scenarios - done
  * Test with new spark max motor controllers
  * Fix issue with IMU not being recognized
  * Test autonomous
  * Test RunToPosition with just odometry
  */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final HazardXbox primary =
      new HazardXbox(Operator.XboxPrimary, Constants.Operator.DeadzoneMin);
  private HazardXbox secondaryControl =
      new HazardXbox(Constants.Operator.XboxSecondary, Constants.Operator.DeadzoneMin);

  // Claw grabbing control
  private Trigger clawOpenTrigger = secondaryControl.leftTrigger();
  private Trigger clawClosedTrigger = secondaryControl.rightTrigger();
  // Claw alingment piston
  // TODO: currently set to the controller dpad, discuss with drive team
  private Trigger clawUpTrigger = secondaryControl.leftBumper();
  private Trigger clawDownTrigger = secondaryControl.rightBumper();
  private Trigger tossTrigger = secondaryControl.povUp();
  // Claw rotation motor
  // private Trigger clawWristLeftTrigger = secondaryControl.leftBumper();
  // private Trigger clawWristRightTrigger = secondaryControl.rightBumper();
  // Arm setpoint buttons
  private Trigger armPosGrabTrigger = secondaryControl.a();
  private Trigger armPosScore1Trigger = secondaryControl.x();
  private Trigger armPosScore2Trigger = secondaryControl.y();
  private Trigger armPosRestTrigger = secondaryControl.b();
  // Resets the arm encoder
  private Trigger armResetTrigger = secondaryControl.back();

  /** Subsystems **/
  private final ArmSubsystem arm = new ArmSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();

  private final BallDrivetrainConfig drivetrainConfig = new BallDrivetrainConfig(
    new Pose2d(new Translation2d(1, 1), new Rotation2d(1, 0)), new Pose2d(new Translation2d(1, -1), new Rotation2d(1, 0)),
    new Pose2d(new Translation2d(-1, 1), new Rotation2d(1, 0)), new Pose2d(new Translation2d(-1, -1), new Rotation2d(1, 0)));
  private final InformationSubsystem info = new InformationSubsystem(new ADIS16470_IMU(), new Pose2d(0, 0, new Rotation2d()));
  //Oops! all spark max
  private final BallDriveSubsystem drivetrain = new BallDriveSubsystem(
    new BallDriveModule(new PWMSparkMax(0), new PWMSparkMax(1), 0.01, true, true),
    new BallDriveModule(new PWMSparkMax(2), new PWMSparkMax(3), 0.01, true, false),
    new BallDriveModule(new PWMSparkMax(4), new PWMSparkMax(5), 0.01, false, true),
    new BallDriveModule(new PWMSparkMax(6), new PWMSparkMax(7), 0.01, false, false),
    drivetrainConfig, info);

  /** Commands **/
  private final ArmCommand armCmd = new ArmCommand(arm, secondaryControl, clawUpTrigger, clawDownTrigger, armPosRestTrigger, armPosScore1Trigger, armPosScore2Trigger, armPosGrabTrigger, armResetTrigger);
  private final ClawToggleCmd clawCmd = new ClawToggleCmd(clawOpenTrigger, clawClosedTrigger, secondaryControl, claw);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Basic Xbox stick controls
    drivetrain.setDisable(new boolean[] {false, false, false, false});
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      drivetrain.set(
        primary.getLeftY(),
        primary.getLeftX(),
        primary.getRightX());
    }, drivetrain));

    arm.setDefaultCommand(armCmd);
    arm.resetEncoder();
    claw.setDefaultCommand(clawCmd);
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* TODO: Autonomous */
    int selection = 1;
    switch (selection) {
      case 0:
      return new SequentialCommandGroup(
        new DriveToPosition(drivetrain, info, new Pose2d(1, 1, new Rotation2d())),
        new DriveToPosition(drivetrain, info, new Pose2d(-10, 10, new Rotation2d(80))),
        new DriveToPosition(drivetrain, info, new Pose2d(0, 0, new Rotation2d(180))));
      case 1:
      return new SequentialCommandGroup(
        new AutonomousCommand(
            drivetrain,
            arm,
            claw,
            info,
            new Pose2d(0, 0, new Rotation2d()),
            2,
            true,
            Constants.Operator.ArmLevelTwoPosition),
        new AutonomousCommand(
            drivetrain,
            arm,
            claw,
            info,
            new Pose2d(0, 0, new Rotation2d()),
            2,
            false,
            Constants.Operator.ArmLevelTwoPosition),
        new AutonomousCommand(
            drivetrain, arm, claw, info, new Pose2d(0.8, 0, new Rotation2d()), 2, false, 0));
      case 2:
      return new SequentialCommandGroup(
        new AutonomousCommand(drivetrain, null, null, info, new Pose2d(new Translation2d(-0.8, 0), new Rotation2d()), 2, false, 0),
        new AutonomousCommand(drivetrain, null, null, info, new Pose2d(new Translation2d(-0.4, 0.4), new Rotation2d()), 2, false, 0),
        new AutonomousCommand(drivetrain, null, null, info, new Pose2d(new Translation2d(0, 0), new Rotation2d()), 2, false, 0),
        new AutonomousCommand(drivetrain, null, null, info, new Pose2d(new Translation2d(-0.2, 0), new Rotation2d()), 1, false, 0)
      );
      default:
      return new SequentialCommandGroup(
        new DriveToPosition(drivetrain, info, new Pose2d(1, 1, new Rotation2d())),
        new DriveToPosition(drivetrain, info, new Pose2d(-10, 10, new Rotation2d(80))),
        new DriveToPosition(drivetrain, info, new Pose2d(0, 0, new Rotation2d(180))));
    }

  }
}
