package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class BallDriveModule {
  private MotorController X;
  private MotorController Y;

  public void set(double x, double y) {
    X.set(x);
    Y.set(y);
  }

  public double[] get() {
    return new double[] {X.get(), Y.get()};
  }

  public Translation2d getVector() {
    return new Translation2d(X.get(), Y.get());
  }
}
