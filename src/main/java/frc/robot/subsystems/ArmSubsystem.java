// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoublePredicate;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {

  private final LoggedTalonFX topLeft, topRight, bottomLeft, bottomRight, master;
  private DutyCycleEncoder revEncoder;
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
    bottomLeft = new LoggedTalonFX(Constants.Arm.bottomLeftMotor.port, canbus);
    master = topLeft;

    revEncoder = new DutyCycleEncoder(Constants.Arm.ENCODER_PORT);


    Follower alignedFollower = new Follower(master.getDeviceID(), MotorAlignmentValue.Aligned);
    Follower opposedFollower = new Follower(master.getDeviceID(), MotorAlignmentValue.Opposed);
    topRight.setControl(opposedFollower);
    bottomRight.setControl(opposedFollower);
    bottomLeft.setControl(opposedFollower);
    

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Arm.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Arm.SUPPLY_CURRENT_LIMIT);

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(0.5)
            .withKI(0)
            .withKD(0)
            .withKV(Constants.Arm.armKV)
            .withKG(Constants.Arm.armKG)
            .withKS(Constants.Arm.armKS);

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

    mmc =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(Constants.Arm.MOTIONMAGIC_KA)
            .withMotionMagicCruiseVelocity(Constants.Arm.MOTIONMAGIC_KV);
    

    topLeft.getConfigurator().apply(clc);
    topRight.getConfigurator().apply(clc);
    bottomLeft.getConfigurator().apply(clc);
    bottomRight.getConfigurator().apply(clc);
    master.getConfigurator().apply(clc);


    topLeft.getConfigurator().apply(s0c);
    topRight.getConfigurator().apply(s0c);
    bottomLeft.getConfigurator().apply(s0c);
    bottomRight.getConfigurator().apply(s0c);
    master.getConfigurator().apply(s0c);


    topLeft.getConfigurator().apply(moc);
    topRight.getConfigurator().apply(moc);
    bottomLeft.getConfigurator().apply(moc);
    bottomRight.getConfigurator().apply(moc);
    master.getConfigurator().apply(moc);


    topLeft.getConfigurator().apply(mmc);
    topRight.getConfigurator().apply(mmc);
    bottomLeft.getConfigurator().apply(mmc);
    bottomRight.getConfigurator().apply(mmc);
    master.getConfigurator().apply(mmc);
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

  public void zeroArm() {
    if (revEncoder.isConnected()) {
      master.setPosition(getAbsolutePos() * Constants.Arm.ARM_CONVERSION_FACTOR);
    }
  }

  public double getAbsolutePos() {
    return (revEncoder.get() - Constants.Arm.ABSOLUTE_ENCODER_HORIZONTAL);
  }

  public boolean atTarget() {
    return Math.abs(
            master.getPosition().getValueAsDouble() * Constants.Arm.ARM_CONVERSION_FACTOR
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

    DogLog.log("Doglog/arm/master StatorCurrent", master.getStatorCurrent().getValueAsDouble());
    DogLog.log("Doglog/arm/bottomleftStatorCurrent", bottomLeft.getStatorCurrent().getValueAsDouble());
    DogLog.log("Doglog/arm/integratedAbsoluteConversion", master.getPosition().getValueAsDouble() * 55.9867);
    DogLog.log("Doglog/arm/integratedConversion", master.getPosition().getValueAsDouble() * 130.63563333333335);
    DogLog.log("Doglog/arm/absoluteConversion", master.getPosition().getValueAsDouble() * (42d / 18d));

    DogLog.log("Doglog/arm/targetDegreesConverted", targetDegrees * Constants.Arm.ARM_CONVERSION_FACTOR);
    DogLog.log("Doglog/arm/getPosition", master.getPosition().getValueAsDouble());

    DogLog.log("Doglog/arm/encoderPos", revEncoder.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
