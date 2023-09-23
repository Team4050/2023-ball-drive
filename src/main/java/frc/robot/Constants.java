// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Copied from Team4050/2023-charged-up, since the ball drive robot will have many similar features
 * 
 */

package frc.robot;

import java.lang.reflect.Type;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public final class Constants {

  public static class Operator {
    public static final int XboxPrimary = 0;
    public static final int XboxSecondary = 1;
    public static final float DeadzoneMin = 0.05f;
    public static final float RotationDamping = 0.5f;

    // TODO: Values to be determined
    // TODO: arm encoder resolution is 4096 per quarter rotation. Calculate and test values for
    // button setpoints.
    public static final int ArmGrabPosition = 5250; // 5000
    public static final int ArmLevelOnePosition = 4300;
    public static final int ArmLevelTwoPosition = 3000;
    public static final int ArmRestPosition = 0;

    // Arm soft limits. 0 is at rest, so we don't want to go too further back.
    public static final int ArmEncoderLimitLow = -10;
    public static final int ArmEncoderLimitHigh = 5379;

    public static final int ArmEncoderBreakLow = -300;
    public static final int ArmEncoderBreakHigh = 5800;

    // Controls the speed at which the manual position control changes
    // TODO: tune
    public static final float ArmJoystickCoefficient = 45;
  }

  public static class Drive {
    public static final Type FL_TYPE = PWMSparkMax.class;
    public static final int FrontLeftX = -1;
    public static final int FrontLeftY = -1;
    public static final Type FR_TYPE = PWMSparkMax.class;
    public static final int FrontRightX = -1;
    public static final int FrontRightY = -1;
    public static final Type RL_TYPE = TalonFX.class;
    public static final int RearLeftX = -1;
    public static final int RearLeftY = -1;
    public static final Type RR_TYPE = TalonFX.class;
    public static final int RearRightX = -1;
    public static final int RearRightY = -1;
  }

  public static class Actuators {
    public static final int Arm = 6;
    public static final int ArmLimitA = 0;
    public static final int ArmLimitB = 0;
    public static final int Wrist = 7;
  }

  public static class Sensors {
    public static final int ClawLimitL = 0;
    public static final int CLawLimitR = 1;
    public static final int ArmLimit = 2;
  }

  public static class Vision {
    public static final String ChassisCamName = "Chassis";
    public static final String ClawCamName = "Claw";
    public static final Transform3d RobotToCamera =
        new Transform3d(new Pose3d(), new Pose3d(1f, 0, 0.2f, new Rotation3d()));
  }

  public static class Pneumatics {
    public static final int PCM = 1;
    public static final PneumaticsModuleType Module = PneumaticsModuleType.CTREPCM;
    public static final int ClawFwdChannel = 0;
    public static final int ClawRevChannel = 1;
    public static final int ArmFwdChannel = 2;
    public static final int ArmRevChannel = 3;
  }
}