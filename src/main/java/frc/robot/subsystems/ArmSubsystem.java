package frc.robot.subsystems;

import java.net.ContentHandler;

import org.opencv.core.Mat;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ArmSubsystem extends SubsystemBase {
  private LoggedTalonFX topLeft, topRight, bottomLeft, bottomRight, master;
  private ArmFeedforward armff;
  private DutyCycleEncoder revEncoder;

  public ArmSubsystem() {
    CurrentLimitsConfigs clc = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.Arm.statorCurrentLimit);
    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    Slot0Configs s0c = new Slot0Configs().withKP(1).withKI(0).withKD(0).withKG(Constants.Arm.armKG)
        .withKS(Constants.Arm.armKS).withKV(Constants.Arm.armKV);

    CANBus canbus = new CANBus("Patrice the Pineapple");

    topLeft = new LoggedTalonFX(Constants.Arm.topLeftMotor.port, canbus);
    topRight = new LoggedTalonFX(Constants.Arm.topRightMotor.port, canbus);
    bottomLeft = new LoggedTalonFX(Constants.Arm.bottomLeftMotor.port, canbus);
    bottomRight = new LoggedTalonFX(Constants.Arm.bottomRightMotor.port, canbus);

    // define followers
    Follower follower = new Follower(Constants.Arm.topLeftMotor.port, MotorAlignmentValue.Aligned);
    Follower invertedFollower = new Follower(Constants.Arm.topLeftMotor.port, MotorAlignmentValue.Opposed);
    topRight.setControl(follower);
    bottomRight.setControl(follower);
    bottomLeft.setControl(invertedFollower);

    master = topLeft;

    TalonFXConfigurator masterConfig = master.getConfigurator();
    TalonFXConfigurator topRightConfig = topRight.getConfigurator();
    TalonFXConfigurator bottomLeftConfig = bottomLeft.getConfigurator();
    TalonFXConfigurator bottomRightConfig = bottomRight.getConfigurator();

    masterConfig.apply(moc);
    topRightConfig.apply(moc);
    bottomLeftConfig.apply(moc);
    bottomRightConfig.apply(moc);

    masterConfig.apply(clc);
    topRightConfig.apply(clc);
    bottomLeftConfig.apply(clc);
    bottomRightConfig.apply(clc);

    MotionMagicConfigs mmc = new MotionMagicConfigs();
    mmc.MotionMagicCruiseVelocity = Constants.Arm.armConversionFactor;
    mmc.MotionMagicAcceleration = 2.2 * Constants.Arm.armConversionFactor;

    masterConfig.apply(s0c);
    masterConfig.apply(mmc);

    revEncoder = new DutyCycleEncoder(0);
  }

  private double getEncoderPos() {
    return (revEncoder.get() + 0.3845) % 1;
  }

  private double getPosDegrees() {
    return (master.getPosition().getValueAsDouble() / Constants.Arm.armConversionFactor) * 360d;
  }

  public void setPosition(double degrees) {
    degrees = MathUtil.clamp(degrees, 3, 110);
    double rawDegrees = (master.getPosition().getValueAsDouble() / Constants.Arm.armConversionFactor) * 360d;
    master.setControl(
        new MotionMagicVoltage(
            490));
  }

  @Override
  public void periodic() {
    DogLog.log("DogLog/arm/encoderPosition", getPosDegrees());
    DogLog.log("DogLog/arm/motorSpeed", master.getVelocity().getValueAsDouble());
    DogLog.log("DogLog/arm/targetPosition", 10);

  }
}
