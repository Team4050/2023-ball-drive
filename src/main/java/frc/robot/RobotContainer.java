// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Operator;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToPosition;
import frc.robot.subsystems.BallDriveModule;
import frc.robot.subsystems.BallDriveSubsystem;
import frc.robot.subsystems.BallDrivetrainConfig;
import frc.robot.subsystems.InformationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
  * Check scenarios
  * Test autonomous
  * Test RunToPosition with just odometry
  */
public class RobotContainer {
  /** Subsystems **/
  private final BallDrivetrainConfig drivetrainConfig = new BallDrivetrainConfig(
    new Pose2d(new Translation2d(-1, -1), new Rotation2d(1, 0)), new Pose2d(new Translation2d(-1, 1), new Rotation2d(0, -1)),
    new Pose2d(new Translation2d(1, -1), new Rotation2d(0, 1)), new Pose2d(new Translation2d(1, 1), new Rotation2d(-1, 0)));
  private final InformationSubsystem info = new InformationSubsystem(new ADIS16470_IMU(IMUAxis.kZ, Port.kMXP, CalibrationTime._512ms), new Pose2d(0, 0, new Rotation2d()));
  //Oops! all spark max
  private final BallDriveSubsystem drivetrain = new BallDriveSubsystem(
    new BallDriveModule(new PWMSparkMax(0), new PWMSparkMax(1), 0.5),
    new BallDriveModule(new PWMSparkMax(2), new PWMSparkMax(3), 0.5),
    new BallDriveModule(new PWMSparkMax(4), new PWMSparkMax(5), 0.5),
    new BallDriveModule(new PWMSparkMax(6), new PWMSparkMax(7), 0.5),
    drivetrainConfig, null);

  /** Commands **/

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController primary =
      new CommandXboxController(Operator.XboxPrimary);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //Basic Xbox stick controls
    drivetrain.setDefaultCommand(new RunCommand(() -> {
      drivetrain.set(
        1,
        0, 
        1);      
    }, drivetrain));
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
    return new SequentialCommandGroup(
      new DriveToPosition(drivetrain, info, new Pose2d(1, 1, new Rotation2d())),
      new DriveToPosition(drivetrain, info, new Pose2d(-10, 10, new Rotation2d(80))),
      new DriveToPosition(drivetrain, info, new Pose2d(0, 0, new Rotation2d(180))));
  }
}
