// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {

  private final LoggedTalonFX topLeft, topRight, bottomLeft, bottomRight, master;
  private static ArmSubsystem instance;
  CANBus canbus = new CANBus("Patrice the Pineapple");

  private MotionMagicConfigs mmc;
  private MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);

  private double targetDegrees = 0;

  private final double tolerance = 1;

  public ArmSubsystem() {
    topLeft = new LoggedTalonFX(Constants.Arm.topLeftMotor.port, canbus);
    topRight = new LoggedTalonFX(Constants.Arm.topRightMotor.port, canbus);
    bottomRight = new LoggedTalonFX(Constants.Arm.bottomRightMotor.port, canbus);
    bottomLeft = new LoggedTalonFX(Constants.Arm.bottomRightMotor.port, canbus);
    master = topLeft;

    Follower alignedFollower = new Follower(master.getDeviceID(), MotorAlignmentValue.Aligned);
    Follower opposedFollower = new Follower(master.getDeviceID(), MotorAlignmentValue.Opposed);
    topRight.setControl(alignedFollower);
    bottomRight.setControl(opposedFollower);
    bottomLeft.setControl(opposedFollower);

    

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Arm.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Arm.SUPPLY_CURRENT_LIMIT);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(1)
            .withKI(0)
            .withKD(0)
            .withKV(Constants.Arm.armKV)
            .withKG(Constants.Arm.armKG)
            .withKS(Constants.Arm.armKS);

    MotorOutputConfigs moc = new MotorOutputConfigs();

    master.getConfigurator().apply(s0c);
    master.getConfigurator().apply(moc);
    master.getConfigurator().apply(clc);

    mmc =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(Constants.Arm.MOTIONMAGIC_KA)
            .withMotionMagicCruiseVelocity(Constants.Arm.MOTIONMAGIC_KV);
  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      instance = new ArmSubsystem();
    }
    return instance;
  }

  public void setPosition(double degrees) {
    targetDegrees = MathUtil.clamp(degrees, 3, 110);

    master.setControl(
        controlRequest.withPosition(targetDegrees / 360 * Constants.Arm.ARM_CONVERSION_FACTOR));
  }

  public void stopArm() {
    master.setControl(controlRequest.withPosition(master.getPosition().getValueAsDouble()));
  }

  public void resetPositionToZero() {
    setPosition(0);
  }

  public double getCurrentDegreePosPerMotor(LoggedTalonFX motor) {
    return motor.getPosition().getValueAsDouble() / 360 * Constants.Arm.ARM_CONVERSION_FACTOR;
  }

  public boolean atTarget() {
    return Math.abs(
            master.getPosition().getValueAsDouble() / 360 * Constants.Arm.ARM_CONVERSION_FACTOR
                - targetDegrees)
        <= tolerance;
  }

  @Override
  public void periodic() {
    DogLog.log("Doglog/arm/targetDegrees", targetDegrees);
    DogLog.log("Doglog/arm/atPosition", atTarget());
    DogLog.log("Doglog/arm/motorlogs/topleftdegree", getCurrentDegreePosPerMotor(topLeft));
    DogLog.log("Doglog/arm/motorlogs/toprightdegree", getCurrentDegreePosPerMotor(topRight));
    DogLog.log("Doglog/arm/motorlogs/bottomleftdegree", getCurrentDegreePosPerMotor(bottomLeft));
    DogLog.log("Doglog/arm/motorlogs/bottomrightdegree", getCurrentDegreePosPerMotor(bottomRight));
    DogLog.log("Doglog/arm/motorlogs/topleftspeed", topLeft.getVelocity().getValueAsDouble());
    DogLog.log("Doglog/arm/motorlogs/toprightspeed", topRight.getVelocity().getValueAsDouble());
    DogLog.log("Doglog/arm/motorlogs/bottomleftspeed", bottomLeft.getVelocity().getValueAsDouble());
    DogLog.log("Doglog/arm/motorlogs/bottomRightspeed", bottomRight.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
