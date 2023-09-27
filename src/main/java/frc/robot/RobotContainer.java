// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Operator;
import frc.robot.commands.Autos;
import frc.robot.subsystems.BallDriveModule;
import frc.robot.subsystems.BallDriveSubsystem;
import frc.robot.subsystems.BallDrivetrainConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /** Subsystems **/
  private final BallDrivetrainConfig drivetrainConfig = new BallDrivetrainConfig(
    new Pose2d(new Translation2d(1, 1), new Rotation2d(180)), new Pose2d(),
    new Pose2d(), new Pose2d());
  private final BallDriveSubsystem drivetrain = new BallDriveSubsystem(
    new BallDriveModule(new PWMSparkMax(0), new PWMSparkMax(1)),
    new BallDriveModule(null, null),
    new BallDriveModule(null, null),
    new BallDriveModule(null, null),
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
        0, 
        -0.01, 
        0);
      System.out.println(drivetrain.get()[0].getX());
      
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
    return null;
  }
}
