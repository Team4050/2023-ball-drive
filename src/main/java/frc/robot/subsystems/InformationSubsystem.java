package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.hazard.HazardVision;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class InformationSubsystem extends SubsystemBase {
  /* Sensors */
  // ADIS winds CCW
  private ADIS16470_IMU imu;

  private Timer timer;
  private HazardVision camera;

  /* Filters & Estimators */
  //TODO: Make ball drive pose estimator
  private BallDriveOdometry drivetrainPoseEstimator;
  private Pose3d estimatedPose;
  private Field2d dashboardField;
  private AprilTagFieldLayout layout;

  public InformationSubsystem(ADIS16470_IMU imu, Pose2d startingPose) {
    this.imu = imu;
    // tab.add("ADIS IMU", imu);
    this.camera = new HazardVision(startingPose);
    // tab.add("Limelight", camera);
    layout = null;

    // Setup field layout from resource file
    try {
      layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      e.printStackTrace();
    }

    //Removed FilteredDrivetrainControl

    //estimatedPose = new Pose3d(drivetrainPoseEstimator.update());
    //dashboardField.setRobotPose(estimatedPose.toPose2d());
    //new Pose3d(startingPose);

    /*Optional<EstimatedRobotPose> p = camera.getEstimatedGlobalPose(startingPose);
    if (p.isPresent()) {
      estimatedPose = p.get().estimatedPose;
    }*/

    camera.claw.setDriverMode(true);

    timer = new Timer();
    timer.reset();
    timer.start();

    dashboardField = new Field2d();
    dashboardField.setRobotPose(startingPose);

    // TODO: okay so this is rotated and reversed, causing AutonomousStep to apply motor power in
    // reverse for strafe & turn
  }

  public void updatePoseEstimate(double dT, double[] encoderPositions) {
    estimatedPose = new Pose3d(drivetrainPoseEstimator.update());
    dashboardField.setRobotPose(estimatedPose.toPose2d());
  }

  public Pose3d getPoseEstimate() {
    return estimatedPose;
  }

  @Override
  public void periodic() {
    if (true) return;
    Optional<EstimatedRobotPose> estimatedPose = camera.getEstimatedGlobalPose(new Pose2d());

    if (estimatedPose.isPresent()) {
      dashboardField.setRobotPose(estimatedPose.get().estimatedPose.toPose2d());
    }
  }

  public enum axis {
    XAcc,
    YAcc,
    ZAcc,
    ZRot,
    ZRate
  }

  /**
   * Pretty self-explainatory. Gets raw data from the IMU.
   *
   * @param a The type of data to return.
   * @return The requested data.
   */
  public double getData(axis a) {
    switch (a) {
      case XAcc:
        return imu.getAccelX();
      case YAcc:
        return imu.getAccelY();
      case ZAcc:
        return imu.getAccelZ();
      case ZRate:
        return imu.getRate();
      case ZRot:
        return imu.getAngle();
      default:
        return 0;
    }
  }

  /**
   * Returns multiple forms of raw IMU data in one call.
   *
   * @param a An array containing axis. The returned array with have corresponding values.
   * @return An array of IMU data. The type of data is requested through the a parameter.
   */
  public double[] getData(axis[] a) {
    double[] r = new double[a.length];
    for (int i = 0; i < a.length; i++) {
      r[i] = getData(a[i]);
    }
    return r;
  }

  /**
   * For autonomous: drives in the supplied direction for the supplied amount of time
   *
   * @param drive The drivetrain subsystem to use.
   * @param direction The direction to drive in.
   * @param timeout The amount of time to drive
   */
  public void driveUntil(BallDriveSubsystem drive, Pose2d direction, int timeout) {
    // 18 degrees / second
    double since = timer.get();
    while (timer.get() < since + timeout) {
      drive.set(
          direction.getX(),
          direction.getY(),
          0); // (direction.getRotation().getRadians() / Math.PI) % 1
    }
  }
}