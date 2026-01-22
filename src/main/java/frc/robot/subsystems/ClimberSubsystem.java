// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Constructor;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalonFX;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants.ClimberPositions;



public class ClimberSubsystem extends SubsystemBase {
  private static ClimberSubsystem instance;
  private final LoggedTalonFX motor1, motor2, master;

  private ClimberPositions currentLevel;
  private final double tolerance = 1;

  private MotionMagicConfigs mmc;
  private final MotionMagicVoltage controlRequest = new MotionMagicVoltage(0);



  public ClimberSubsystem() {
    motor1 = new LoggedTalonFX(Constants.ClimberConstants.MOTOR1_PORT, Constants.ClimberConstants.canbus);
    motor2 = new LoggedTalonFX(Constants.ClimberConstants.MOTOR2_PORT, Constants.ClimberConstants.canbus);
    master = motor1;

    Follower follower = new Follower(Constants.ClimberConstants.MOTOR1_PORT, MotorAlignmentValue.Opposed);
    motor2.setControl(follower);


    Slot0Configs s0c = new Slot0Configs().withKP(Constants.ClimberConstants.S0C_KP).withKI(Constants.ClimberConstants.S0C_KI).withKD(Constants.ClimberConstants.S0C_KD).withKS(Constants.ClimberConstants.S0C_KS).withKG(Constants.ClimberConstants.S0C_KG).withKA(Constants.ClimberConstants.S0C_KA).withKV(Constants.ClimberConstants.S0C_KV);
  
    motor1.updateCurrentLimits(Constants.ClimberConstants.STATOR_CURRENT_LIMIT, Constants.ClimberConstants.SUPPLY_CURRENT_LIMIT);
    motor2.updateCurrentLimits(Constants.ClimberConstants.STATOR_CURRENT_LIMIT, Constants.ClimberConstants.SUPPLY_CURRENT_LIMIT);

    mmc = new MotionMagicConfigs();
    mmc.MotionMagicAcceleration = Constants.ClimberConstants.MOTIONMAGIC_MAX_ACCELERATION;
    mmc.MotionMagicCruiseVelocity = Constants.ClimberConstants.MOTIONMAGIC_MAX_VELOCITY;

    TalonFXConfigurator m1config = motor1.getConfigurator();
    TalonFXConfigurator m2config = motor2.getConfigurator();

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    
    m1config.apply(s0c);
    m2config.apply(s0c);
    m1config.apply(mmc);
    m2config.apply(mmc);
    m1config.apply(moc);
    m2config.apply(moc);
  }

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      instance = new ClimberSubsystem();
    }
    return instance;
  }

  public void setClimbTo(ClimberPositions level) {
   this.currentLevel = level;
   master.setControl(controlRequest.withPosition(level.height * Constants.ClimberConstants.CONVERSOIN_DISTANCE_TO_ROTATIONS));
  }

  public double getError() {
    return (currentLevel.height * Constants.ClimberConstants.CONVERSOIN_DISTANCE_TO_ROTATIONS) - master.getPosition().getValueAsDouble();
    // returns in rotations
  }

  public boolean isAtLevel() {
    return Math.abs(getError()) <= tolerance;
  }
 

  @Override
  public void periodic() {
    DogLog.log("Doglog/climber/error", getError());
    DogLog.log("Doglog/climber/target", currentLevel.getHeight());
    DogLog.log("Doglog/climber/isAtLevel", isAtLevel());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
