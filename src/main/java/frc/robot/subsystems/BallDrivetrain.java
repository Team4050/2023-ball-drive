package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
/*import cshcyberhawks.swolib.*;
import cshcyberhawks.swolib.swerve.SwerveDriveTrain;
import cshcyberhawks.swolib.swerve.configurations.FourWheelSwerveConfiguration;
import cshcyberhawks.swolib.swerve.configurations.SwerveModuleConfiguration;*/
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallDrivetrain extends SubsystemBase {
  /*
   * Front Sparkmax drive motors
   */
  //#region
  private BallDriveModule FL;
  private BallDriveModule FR;
  private BallDriveModule RL;
  private BallDriveModule RR;

  private BallDrivetrainConfig config;
  //#endregion

  //SwerveDriveTrain driveTrain;

  public BallDrivetrain(BallDriveModule FL, BallDriveModule FR, BallDriveModule RL, BallDriveModule RR, BallDrivetrainConfig config) {
    this.FL = FL;
    this.FR = FR;
    this.RL = RL;
    this.RR = RR;
    this.config = config;
    //SwerveDriveKinematics k = new SwerveDriveKinematics(null);
    //FourWheelSwerveConfiguration config = new FourWheelSwerveConfiguration(null, null, null, null, null, null);
    //driveTrain = new SwerveDriveTrain(config, null);
  }

  /*
   * Sets drivetrain speed
   */
  public void set(double x, double y, double a) {
    Translation2d V = new Translation2d(x, y);
    Translation2d FLV = config.FLTangent;
    FLV.times(a);
    FLV.plus(V);
    FLV = config.FromRefFrame(FLV, 0);
    FL.set(FLV.getX(), FLV.getY());

    /* TODO: test FL and if it works create code for all other modules */
  }

  /*
   * Uses odometry to set the drivetrain speed relative to the robot's starting pose
   */
  public void setAbsolute(float x, float y, float a) {

  }


  public Translation2d[] get() {
    double[] fl = FL.get();
    double[] fr = FR.get();
    double[] rl = RL.get();
    double[] rr = RR.get();
    return new Translation2d[] {new Translation2d(fl[0], fl[1]), new Translation2d(fr[0], fr[1]), new Translation2d(rl[0], rl[1]), new Translation2d(rr[0], rr[1])};
  }
}
