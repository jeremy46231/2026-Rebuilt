// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTalonFX;

public class ShooterSubsystem extends SubsystemBase {
  private final LoggedTalonFX warmUpMotor1, warmUpMotor2, warmUpMotor3, shooter;

  private TalonFXSimState warmUpMotor1SimState, warmUpMotor2SimState, warmUpMotor3SimState;
  private DCMotorSim warmUpMotor1Sim, warmUpMotor2Sim, warmUpMotor3Sim;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

  private static double targetSpeed = 0;

  public ShooterSubsystem() {

    warmUpMotor1 = new LoggedTalonFX(Constants.Shooter.WARMUP_MOTOR_1_ID);
    warmUpMotor2 = new LoggedTalonFX(Constants.Shooter.WARMUP_MOTOR_2_ID);
    warmUpMotor3 = new LoggedTalonFX(Constants.Shooter.WARMUP_MOTOR_3_ID);

    Follower follower =
        new Follower(Constants.Shooter.WARMUP_MOTOR_1_ID, MotorAlignmentValue.Aligned);
  
    warmUpMotor2.setControl(follower);
    warmUpMotor3.setControl(follower);
    shooter = warmUpMotor1;

    Slot0Configs s0c =
        new Slot0Configs()
            .withKP(Constants.Shooter.KP)
            .withKI(Constants.Shooter.KI)
            .withKD(Constants.Shooter.KD)
            .withKV(Constants.Shooter.KV)
            .withKA(Constants.Shooter.KA);

    CurrentLimitsConfigs clc =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(Constants.Shooter.STATOR_CURRENT_LIMIT)
            .withSupplyCurrentLimit(Constants.Shooter.SUPPLY_CURRENT_LIMIT);

    TalonFXConfigurator m1config = warmUpMotor1.getConfigurator();
    TalonFXConfigurator m2config = warmUpMotor2.getConfigurator();
    TalonFXConfigurator m3config = warmUpMotor3.getConfigurator();

    m1config.apply(s0c);
    m2config.apply(s0c);
    m3config.apply(s0c);
    m1config.apply(clc);
    m2config.apply(clc);
    m3config.apply(clc);
  }

  public double calculateFtToRPS(double speed) {
    return speed
        / (Constants.Shooter.SHOOTER_WHEEL_DIAMETER_IN * Math.PI / 12)
        * Constants.Shooter.MOTOR_ROTS_PER_SHOOTER_ROTS;
    // from surface speed in ft/sec to rps
  }

  public double calculateRPSToFt(double motorRPS) {
      // Convert motor RPS to shooter wheel RPS
      double shooterWheelRPS = motorRPS / Constants.Shooter.MOTOR_ROTS_PER_SHOOTER_ROTS;
      // Convert shooter wheel RPS to surface speed
      return shooterWheelRPS * (Constants.Shooter.SHOOTER_WHEEL_DIAMETER_IN * Math.PI / 12);
  }

  // speed based on shooter wheel which is the one flinging the ball with a max of 52.36 and a min
  // of 35.60 ft/sec
  // input the speed you want the ball to go at (ft/sec); it will be divided by 2 because that's
  // what Jeff said that relationship is
  // so now max is 104.72 and min is 71.2
  public void setSpeed(double speed) {
    targetSpeed = speed / 2;
    shooter.setControl(velocityRequest.withVelocity(calculateFtToRPS(targetSpeed)));
  }

  public void stop() {
    setSpeed(0);
  }

  public boolean isAtSpeed() {
    return Math.abs(calculateFtToRPS(targetSpeed) - shooter.getVelocity().getValueAsDouble())
        <= Constants.Shooter.SHOOTER_SPEED_TOLERANCE_RPS;
  }

  public double getCurrentSpeed() {
    return calculateRPSToFt(shooter.getVelocity().getValueAsDouble());
  }

  // Comands
  public Command ShootAtSpeed() {
    return Commands.runEnd(() -> this.setSpeed(Constants.Shooter.SHOOT_FOR_AUTO), this::stop, this);
  }

  @Override
  public void periodic() {
    DogLog.log("Doglog/shooter/targetSpeed", targetSpeed);
    DogLog.log("Doglog/shooter/isAtSpeed", isAtSpeed());
    DogLog.log("Doglog.shooter/currentSpeed", getCurrentSpeed());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}