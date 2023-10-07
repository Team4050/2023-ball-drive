package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class BallDriveModule {
  private MotorController X;
  private MotorController Y;
  private double xT, yT = 0;
  private double maxStep = 0.5;

  /**
   * Constructs a ball drive module from two motors, X & Y
   * @param X The motor on the left of the module, looking forward
   * @param Y The motor on the back side of the module, looking forward
   */
  public BallDriveModule(MotorController X, MotorController Y, double maxStep) {
    this.X = X;
    this.Y = Y;
    this.maxStep = maxStep;
  }

  /**
   * Equivalent to MotorController.set() for both axis
   * @param x
   * @param y
   */
  public void set(double x, double y) {
    if (x - xT > maxStep) xT += maxStep;
    else if (x - xT < -maxStep) xT -= maxStep;
    else xT = x;

    if (y - yT > maxStep) yT += maxStep;
    else if (y - yT < -maxStep) yT -= maxStep;
    else yT = y;

    limitSpeed(1);

    X.set(xT);
    Y.set(yT);
  }

  /**
   * Equivalent to MotorController.get() for both axis
   * @return
   */
  public double[] get() {
    return new double[] {X.get(), Y.get()};
  }

  /**
   * Returns a vector determined by the get() functions of motors X & Y
   * @return
   */
  public Translation2d getVector() {
    return new Translation2d(X.get(), Y.get());
  }

  public void limitSpeed(double speed) {
    double mag = Math.hypot(xT, yT);
    if (mag > speed) {
      mag = speed / mag;
      xT *= mag;
      yT *= mag;
    }
  }
}
