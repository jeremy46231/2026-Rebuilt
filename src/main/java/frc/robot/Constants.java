// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Landmarks {

    public static final double MIDLINE_X = 0;
  }

  public static final class Shooter {
    public static final MotorConstants motor1Constants = new MotorConstants(30);
    public static final MotorConstants motor2Constants = new MotorConstants(31);
    public static final MotorConstants preShooterConstants = new MotorConstants(32);

    public static final int ObjectDetectorPort = 1;
  }

  public static final class Arm {
    public static final MotorConstants topLeftMotor = new MotorConstants(12);
    public static final MotorConstants topRightMotor = new MotorConstants(14);
    public static final MotorConstants bottomLeftMotor = new MotorConstants(11);
    public static final MotorConstants bottomRightMotor = new MotorConstants(13);
    public static final int ENCODER_PORT = 0;

    public static final double armKS = 0.16969;
    public static final double armKG = 0.34;
    public static final double armKV = 2.49;

    public static final double MOTIONMAGIC_KV = 1;
    public static final double MOTIONMAGIC_KA = 2.2;

    public static final double STATOR_CURRENT_LIMIT = 40.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;

    public static final double ARM_CONVERSION_FACTOR = (42d / 18d) * 55.9867; // 130.63563333333335

    public static final double ABSOLUTE_ENCODER_HORIZONTAL = 0.6655; // 0.6547
    public static final double ABSOLUTE_HORIZONTAL_OFFSET = 0.05; // 0.05
  }

  public static class MotorConstants {
    public int port;

    public MotorConstants(int port) {
      this.port = port;
    }
  }
}
