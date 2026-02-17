package frc.robot;

import java.util.ArrayList;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoRoutines {
  final AutoTrajectory moveForward;
  final AutoRoutine routine;
  private SequentialCommandGroup cmdgrp;

  public AutoRoutines(AutoFactory factory, boolean choice1, boolean choice2) {
    routine = factory.newRoutine("CristianoRonaldo.chor");
    moveForward = routine.trajectory("MoveForward.traj");

    SequentialCommandGroup cmdgrp = new SequentialCommandGroup();
    //TODO: do this but like good actually
    cmdgrp.addCommands(routine.trajectory("PushForward").cmd());
    if (choice1) {
      cmdgrp.addCommands(routine.trajectory("rIntake").cmd());
    } else {
      cmdgrp.addCommands(routine.trajectory("lIntake").cmd());
    }

    if (choice2) {
      cmdgrp.addCommands(routine.trajectory("rSkeddadleBack").cmd());
    } else {
      cmdgrp.addCommands(routine.trajectory("lSkeddadleBack").cmd());
    }

    routine.active().onTrue(moveForward.resetOdometry().andThen(cmdgrp));
  }

  public AutoRoutine moveForwardAuto() {
    return routine;
  }

  public Command getPathAsCommand() {
    return cmdgrp;
  }
}
