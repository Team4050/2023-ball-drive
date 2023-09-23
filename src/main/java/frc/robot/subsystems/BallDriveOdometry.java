package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import frc.robot.hazard.HazardVision;

public class BallDriveOdometry {
  private BallDrivetrain drivetrain;
  private ADIS16448_IMU imu;
  private HazardVision vision;

  public BallDriveOdometry(BallDrivetrain drivetrain, ADIS16448_IMU imu, HazardVision vision) {
    this.drivetrain = drivetrain;
    this.imu = imu;
    this.vision = vision;
  }

  /**
   * Updates and estimates the robot's absolute position.
   * @return The robot's estimated pose
   */
  public Pose2d update() {
    return null;
    /* TODO: figure out odometry again :\ */
  }
}
