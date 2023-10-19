package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  private DoubleSolenoid piston;

  private Value target;

  public ClawSubsystem() {
    piston =
        new DoubleSolenoid(
            Constants.Pneumatics.PCM,
            Constants.Pneumatics.Module,
            Constants.Pneumatics.ClawFwdChannel,
            Constants.Pneumatics.ClawRevChannel);
  }

  /**
   * Sets the target value for the double solenoid
   *
   * @param state The target value (Forward, Reverse, or Off)
   */
  public void setTargetState(Value state) {
    target = state;
  }

  /** Powers the piston, moving it to the target state */
  public void activate() {
    piston.set(target);
  }
}