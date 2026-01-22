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

  public static class ClimberConstants {
    public static final int MOTOR1_PORT = 11;
    public static final int MOTOR2_PORT = 12;
    public static final int CANRANGE_PORT = 41;
    public static final int kDriverControllerPort = 0;
    public static final double STATOR_CURRENT_LIMIT = 50.0;
    public static final double SUPPLY_CURRENT_LIMIT = 30.0;

    public static double S0C_KP = 1.04; // 1.0 before (okay)
    public static double S0C_KI = 0.00;
    public static double S0C_KD = 0.005;
    public static double S0C_KS = 0.00;
    public static double S0C_KG = 0.29 + 0.05;
    public static double S0C_KA = 0.0004657452997; // 0.04
    public static double S0C_KV = 0.124; // 10.66

    public static final double MOTIONMAGIC_MAX_VELOCITY = 60;
    public static final double MOTIONMAGIC_MAX_ACCELERATION = 60;

    public static final double CONVERSOIN_DISTANCE_TO_ROTATIONS = 12d / ((1.751 * Math.PI) * 0.0254); // sproket gear ratio divided by sproket circum inches * 0.0254

    public static enum ClimberPositions {
      L1(1, 0),
      L2(2, 0),
      L3(3, 0);


      public final int position;
      public final double height;

      ClimberPositions(int pos, double height) {
        this.position = pos;
        this.height = height;
      }

      public int getPosition() {
        return this.position;
      }

      public double getHeight() {
        return this.height;
      }
    }
  }
}


