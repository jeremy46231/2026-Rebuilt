package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRoutines {
  private final AutoFactory factory;
  private final IntakeSubsystem intake;
  private final ShooterSubsystem lebron;
  private final HopperSubsystem hopper;


  public AutoRoutines(AutoFactory factory, IntakeSubsystem intake, ShooterSubsystem lebron, HopperSubsystem hopper) {
    this.factory = factory;
    this.intake = intake;
    this.lebron = lebron;
    this.hopper = hopper;
  }
  
  private AutoTrajectory maneuver(AutoRoutine routine, String name) {
    AutoTrajectory traj;

    switch (name) {
      case "LeftManeuverL":
        traj = routine.trajectory("LeftManeuverL");
        break;
      case "LeftManeuverR":
        traj = routine.trajectory("LeftManeuverR");
        break;
      case "RightManeuverL":
        traj = routine.trajectory("RightManeuverL");
        break;
      case "RightManeuverR":
        traj = routine.trajectory("RightManeuverR");
        break;
      default:
        traj = null;
        break;
    }

    return traj;
  }

  private AutoTrajectory 






  public AutoRoutine moveForwardAuto() {
    return routine;
  }

  public Command getPathAsCommand() {
    return routine.cmd();
  }
}
