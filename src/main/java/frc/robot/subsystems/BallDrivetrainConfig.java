package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

public class BallDrivetrainConfig {
  private Matrix fl;
  private Matrix flTranspose;
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

  public BallDrivetrainConfig(Pose2d FL, Pose2d FR, Pose2d RL, Pose2d RR) {
    Rotation2d deg90 = new Rotation2d(Math.PI * 0.5);
    this.FL = FL;
    FLTranspose = new Translation2d(1, new Rotation2d(-FL.getRotation().getDegrees()));
    FLTangent = new Translation2d(FL.getX(), FL.getY()).rotateBy(deg90);
    this.FR = FR;
    FRTranspose = new Translation2d(1, new Rotation2d(-FR.getRotation().getDegrees()));
    this.RL = RL;
    this.RR = RR;
  }

  public Translation2d FromRefFrame(Translation2d norm, int motor) {
    Rotation2d rot;
    switch (motor) {
      case 0:
        rot = FLTranspose.getAngle();
        break;
      default:
        rot = FLTranspose.getAngle();
        break;
    }
    norm = norm.rotateBy(rot);
    return norm;
  }
}
