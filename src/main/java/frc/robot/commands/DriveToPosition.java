package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.BallDriveSubsystem;
import frc.robot.subsystems.InformationSubsystem;
import java.util.HashSet;
import java.util.Set;

public class DriveToPosition extends CommandBase {
  private BallDriveSubsystem drive;
  private InformationSubsystem info;
  private Pose2d dest;
  private PIDController X;
  private PIDController Y;
  private ProfiledPIDController Rotation;
  private HolonomicDriveController controller;
  private Timer timer = new Timer();
  private Set<Subsystem> reqs;

  public DriveToPosition(BallDriveSubsystem drive, InformationSubsystem info, Pose2d dest) {
    this.drive = drive;
    this.info = info;
    this.dest = dest;
    HashSet<Subsystem> set = new HashSet<Subsystem>();
    set.add(drive);
    reqs = set;
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds =
        controller.calculate(
            info.getPoseEstimate().toPose2d(), new State(1, 0, 0, dest, 0), dest.getRotation());
    // System.out.println(trajectory.sample(timer.get()).poseMeters);
    System.out.println(
        String.format(
            "%f, %f, %f",
            controller.getXController().getPositionError(),
            controller.getYController().getPositionError(),
            controller.getThetaController().getPositionError()));
    //TODO: tune
    drive.setAbsolute(
        speeds.vxMetersPerSecond / 15,
        -speeds.vyMetersPerSecond / 15,
        -speeds.omegaRadiansPerSecond / 15);
  }

  @Override
  public void initialize() {
    X = new PIDController(0.5, 0.08, 0.1);
    Y = new PIDController(0.5, 0.08, 0.1);
    Rotation = new ProfiledPIDController(0.5, 0.1, 0.1, new Constraints(1, 0.5));
    controller = new HolonomicDriveController(X, Y, Rotation);
    // Tolerance of +-5cm & +-5 degrees
    controller.setTolerance(new Pose2d(0.05, 0.05, new Rotation2d(Math.PI / 288)));
    timer = new Timer();
    timer.start();
  }

  @Override
  public Set<Subsystem> getRequirements() {
    return reqs;
  }

  @Override
  public boolean isFinished() {
    return controller.atReference();
  }
}
