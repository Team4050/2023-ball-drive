package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BallDrivetrainConfig {
  public Pose2d FL;
  public Translation2d FLTranspose;
  public Translation2d FLTangent;

  public Pose2d FR;
  public Translation2d FRTranspose;
  public Translation2d FRTangent;

  public Pose2d RL;
  public Translation2d RLTranspose;
  public Translation2d RLTangent;

  public Pose2d RR;
  public Translation2d RRTranspose;
  public Translation2d RRTangent;

  /**
   * A class which holds information as to the locations of the modules relative to the center of the robot,
   * as well as functions to shift frames of reference.
   * @param FL
   * @param FR
   * @param RL
   * @param RR
   */
  public BallDrivetrainConfig(Pose2d FL, Pose2d FR, Pose2d RL, Pose2d RR) {
    Rotation2d deg90 = new Rotation2d(Math.PI * 0.5);
    this.FL = FL;
    FLTranspose = new Translation2d(1, new Rotation2d(-FL.getRotation().getDegrees()));
    FLTangent = new Translation2d(FL.getX(), FL.getY()).rotateBy(deg90);
    this.FR = FR;
    FRTranspose = new Translation2d(1, new Rotation2d(-FR.getRotation().getDegrees()));
    FRTangent = new Translation2d(FR.getX(), FR.getY()).rotateBy(deg90);
    this.RL = RL;
    RLTranspose = new Translation2d(1, new Rotation2d(-FR.getRotation().getDegrees()));
    RLTangent = new Translation2d(RL.getX(), RL.getY()).rotateBy(deg90);
    this.RR = RR;
    RRTranspose = new Translation2d(1, new Rotation2d(-RR.getRotation().getDegrees()));
    RRTangent = new Translation2d(RR.getX(), RR.getY()).rotateBy(deg90);
  }

  /**
   * Transforms a robot-relative vector to a ball module-relative vector
   * @param vector The robot-relative vector to transform
   * @param module The frame of reference to transform to. (0 to 3, Front left, front right, rear left, rear right. Default returns front left)
   * @return
   */
  public Translation2d FromRefFrame(Translation2d vector, int module) {
    Rotation2d rot;
    switch (module) {
      case 0:
        rot = FLTranspose.getAngle();
        break;
      case 1:
        rot = FRTranspose.getAngle();
        break;
      case 2:
        rot = RLTranspose.getAngle();
        break;
      case 3:
        rot = RRTranspose.getAngle();
        break;
      default:
        rot = FLTranspose.getAngle();
        break;
    }
    vector = vector.rotateBy(rot);
    return vector;
  }
}
