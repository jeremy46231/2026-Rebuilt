// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class LEDsubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private CANdle candle;

  private CANdleConfiguration config;

  private CommandXboxController controller;

  private int r, g, b;

  public LEDsubsystem(CommandXboxController controller) {
    candle = new CANdle(39);
    config =
        new CANdleConfiguration()
            .withLED(new LEDConfigs().withStripType(StripTypeValue.RGB).withBrightnessScalar(.5));

    this.controller = controller;
  }

  public CANdle getCandle() {
    return candle;
  }

  public void setLEDcontrol(int red, int green, int blue) {
    r = red;
    g = green;
    b = blue;
  }

  public int rChannel() {
    return r;
  }

  public int gChannel() {
    return g;
  }

  public int bChannel() {
    return b;
  }

  @Override
  public void periodic() {
    if (controller.x().getAsBoolean()) {
      setLEDcontrol(0, 0, 255);
    } else {
      setLEDcontrol(0, 0, 255);
    }
  }
}
