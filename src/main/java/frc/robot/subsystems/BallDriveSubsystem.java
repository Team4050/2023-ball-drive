package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
/*import cshcyberhawks.swolib.*;
import cshcyberhawks.swolib.swerve.SwerveDriveTrain;
import cshcyberhawks.swolib.swerve.configurations.FourWheelSwerveConfiguration;
import cshcyberhawks.swolib.swerve.configurations.SwerveModuleConfiguration;*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallDriveSubsystem extends SubsystemBase {
  /*
   * Drive modules & module config
   */
  private BallDriveModule FL, FR, RL, RR;
  private boolean[] disable;
  private BallDrivetrainConfig config;

  private double xT, yT, aT = 0;
  private double aScale = 0.5;
  private double maxXStep, maxYStep, maxAStep = 10; //Tied to loop cycle time, need to test to find optimal step speed

  private InformationSubsystem info;

  /**
   * A drivetrain with balls. Control inputs are very similar to mecanum.
   * This might backport well to old mecanum drive code.
   * @param FL
   * @param FR
   * @param RL
   * @param RR
   * @param config The drivetrain's configuration, probably the biggest difference from mecanum code.
   */
  public BallDriveSubsystem(BallDriveModule FL, BallDriveModule FR, BallDriveModule RL, BallDriveModule RR, BallDrivetrainConfig config, InformationSubsystem info) {
    this.FL = FL;
    this.FR = FR;
    this.RL = RL;
    this.RR = RR;
    this.config = config;
    maxXStep = 10;
    maxYStep = 10;
    maxAStep = 10;
    disable = new boolean[4];
    disable[0] = false;
    disable[1] = false;
    disable[2] = false;
    disable[3] = false;
    //SwerveDriveKinematics k = new SwerveDriveKinematics(null);
    //FourWheelSwerveConfiguration config = new FourWheelSwerveConfiguration(null, null, null, null, null, null);
    //driveTrain = new SwerveDriveTrain(config, null);
  }
  /**
   * Sets the drivetrain's target speed with a limit on changes in speed.
   * @param x Forward/back target speed
   * @param y Strafing target speed
   * @param a Turning target speed
   */
  public void set(double x, double y, double a) {
    if (x - xT > maxXStep) xT += maxXStep;
    else if (x - xT < -maxXStep) xT -= maxXStep;
    else xT = x;

    if (y - yT > maxYStep) yT += maxYStep;
    else if (y - yT < -maxYStep) yT -= maxYStep;
    else yT = y;

    if (a - aT > maxAStep) aT += maxAStep;
    else if (a - aT < -maxAStep) aT -= maxAStep;
    else aT = a;

    setUnlimited(xT, yT, aT * aScale);
  }


  /**
   * Sets the drivetrain's speed to the target speed
   * @param x Forward/back speed
   * @param y Strafing speed
   * @param a Turning speed
   */
  public void setUnlimited(double x, double y, double a) {
    Translation2d V = new Translation2d(x, y);
    if (!disable[0]) {
      Translation2d FLV = config.FLTangent;
      FLV = FLV.times(a);
      FLV = FLV.plus(V);
      FLV = config.FromRefFrame(FLV, 0);
      FL.set(FLV.getX(), FLV.getY());
    }

    if (!disable[1]) {
      Translation2d FRV = config.FRTangent;
      FRV = FRV.times(a);
      FRV = FRV.plus(V);
      FRV = config.FromRefFrame(FRV, 1);
      FR.set(FRV.getX(), FRV.getY());
    }

    if (!disable[2]) {
      Translation2d RLV = config.RLTangent;
      RLV = RLV.times(a);
      RLV = RLV.plus(V);
      RLV = config.FromRefFrame(RLV, 2);
      RL.set(RLV.getX(), RLV.getY());
    }

    if (!disable[3]) {
      Translation2d RRV = config.RRTangent;
      RRV = RRV.times(a);
      RRV = RRV.plus(V);
      RRV = config.FromRefFrame(RRV, 3);
      RR.set(RRV.getX(), RRV.getY());
    }
  }

  public void setDisable(boolean[] n) {
    disable = n;
  }

  /**
   * Uses odometry to set the drivetrain's target speed relative to the robot's starting position
   * @param x Absolute velocity in the X direction
   * @param y Absolute velocity in the Y direction
   * @param a Turning speed
   */
  public void setAbsolute(double x, double y, double a) {
    Rotation2d rotation = info.getPoseEstimate().toPose2d().getRotation();
    Translation2d vector = new Translation2d(x, y);
    vector.rotateBy(rotation.unaryMinus());
    set(vector.getX(), vector.getY(), a);
  }

  /**
   * Gets an array of robot-relative vectors that correspond to the module's velocities
   * @return An ordered array of velocities: {Front left, front right, rear left, rear right}
   */
  public Translation2d[] get() {
    return new Translation2d[] {
      FL.getVector().rotateBy(config.FL.getRotation()),
      FR.getVector().rotateBy(config.FR.getRotation()),
      RL.getVector().rotateBy(config.RL.getRotation()),
      RR.getVector().rotateBy(config.RR.getRotation())};
  }
}
