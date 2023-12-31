package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import frc.robot.hazard.HazardVision;

public class BallDriveOdometry {
  private BallDriveSubsystem drivetrain;
  private ADIS16448_IMU imu;
  private HazardVision vision;
  /** The kinematics are the same for swerve drive */
  private SwerveDriveKinematics kinematics;
  private SwerveDrivePoseEstimator estimator;
  private Pose2d previous = new Pose2d();

  public BallDriveOdometry(BallDriveSubsystem drivetrain, BallDrivetrainConfig config, ADIS16448_IMU imu, HazardVision vision) {
    this.drivetrain = drivetrain;
    this.imu = imu;
    this.vision = vision;
    kinematics = new SwerveDriveKinematics(
      config.FL.getTranslation(),
      config.FR.getTranslation(),
      config.RL.getTranslation(),
      config.RR.getTranslation());
    estimator = new SwerveDrivePoseEstimator(kinematics, getAngle(), null, previous, null, null);
  }

  /**
   * Updates and estimates the robot's absolute position.
   * @return The robot's estimated pose
   */
  public Pose2d update() {
    Translation2d[] measuredVectors = drivetrain.get();
    Translation2d[] projectedVectors = new Translation2d[4];
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = new SwerveModulePosition(measuredVectors[0].getNorm(), new Rotation2d(measuredVectors[0].getX(), measuredVectors[0].getY()));
    positions[1] = new SwerveModulePosition(measuredVectors[1].getNorm(), new Rotation2d(measuredVectors[1].getX(), measuredVectors[1].getY()));
    positions[2] = new SwerveModulePosition(measuredVectors[2].getNorm(), new Rotation2d(measuredVectors[2].getX(), measuredVectors[2].getY()));
    positions[3] = new SwerveModulePosition(measuredVectors[3].getNorm(), new Rotation2d(measuredVectors[3].getX(), measuredVectors[3].getY()));
    if (vision.isTrustworthy()) {
      estimator.addVisionMeasurement(vision.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), 0);
    }
    previous = estimator.update(new Rotation2d(imu.getAngle()), positions);
    return previous;
  }

  /**
   * Returns the imu.getAngle() but in a Rotation2d.
   * @return
   */
  public Rotation2d getAngle() {
    return new Rotation2d(Math.toRadians(imu.getAngle()));
  }
}
