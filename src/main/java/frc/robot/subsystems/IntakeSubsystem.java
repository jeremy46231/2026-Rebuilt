// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private LoggedTalonFX motor1;
  private LoggedTalonFX motor2;
  public LoggedTalonFX master;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

  public IntakeSubsystem() {
    // Initialize motors
    motor1 =
        new LoggedTalonFX(
            "subsystems/Intake/motor1",
            Constants.Intake.MOTOR1_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);
    motor2 =
        new LoggedTalonFX(
            "subsystems/Intake/motor1",
            Constants.Intake.MOTOR2_PORT,
            Constants.Swerve.WHICH_SWERVE_ROBOT.CANBUS_NAME);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Intake.KP);
            .withKI(Constants.Intake.KI);
            .withKD(Constants.Intake.KD);

    // Set up motor followers and deal with inverted motors
    Follower follower = new Follower(Constants.Intake.MOTOR1_PORT, MotorAlignmentValue.Aligned);
    motor2.setControl(follower);
  }

  public void setProportion(double dutyCycleProportion) {
    motor1.setControl(dutyCycleRequest.withOutput(dutyCycleProportion));
  }

  public void setVelocity(double motorVelocity) {
    motor1.setControl(velocityRequest.withVelocity(motorVelocity));
  }

  @Override
  public void periodic() {
  }
}
