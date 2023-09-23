package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class BallDriveModule {
  private MotorController X;
  private MotorController Y;

  /**
   * Constructs a ball drive module from two motors, X & Y
   * @param X The motor on the left of the module, looking forward
   * @param Y The motor on the back side of the module, looking forward
   */
  public BallDriveModule(MotorController X, MotorController Y) {
    this.X = X;
    this.Y = Y;
  }

  /**
   * Equivalent to MotorController.set() for both axis
   * @param x
   * @param y
   */
  public void set(double x, double y) {
    X.set(x);
    Y.set(y);
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
}
