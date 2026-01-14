package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalonFX;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;
  private LoggedTalonFX motor;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  ;

  public IntakeSubsystem() {
    // change ports as needed
    motor = new LoggedTalonFX(33);

    Slot0Configs s0c = new Slot0Configs();

    MotorOutputConfigs moc = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);
    MotionMagicConfigs mmc = new MotionMagicConfigs();

    motor.updateCurrentLimits(50, 30);

    TalonFXConfigurator mConfig = motor.getConfigurator();

    mConfig.apply(s0c);
    mConfig.apply(moc);
    mConfig.apply(mmc);
  }

  public static IntakeSubsystem getInstance() {
    if (instance == null) instance = new IntakeSubsystem();
    return instance;
  }

  public void setSpeed(double speed) {
    motor.setControl(velocityRequest.withVelocity(speed));
  }

  public void stop() {
    motor.setControl(velocityRequest.withVelocity(0));
  }

  @Override
  public void periodic() {
    DogLog.log("intakeMotor", motor.getVelocity().getValueAsDouble());
  }
}
