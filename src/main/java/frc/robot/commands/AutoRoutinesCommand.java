package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutinesCommand extends SequentialCommandGroup {
  final AutoTrajectory moveForward;
  final AutoRoutine routine;

  public AutoRoutinesCommand(AutoFactory factory) {
    routine = factory.newRoutine("CristianoRonaldo.chor");
    moveForward = routine.trajectory("MoveForward.traj");

    addCommands(moveForward.resetOdometry(), moveForward.cmd());
  }

  public AutoRoutine moveForwardAuto() {
    return routine;
  }

  public Command getPathAsCommand() {
    return routine.cmd();
  }
}
