package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveToPose;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonL1Climb extends SequentialCommandGroup {
  public AutonL1Climb(ClimberSubsystem climberSubsystem, CommandSwerveDrivetrain swerveDrivetrain) {
    addCommands(climberSubsystem.L1Climb(), new DriveToPose(swerveDrivetrain));
  }
}
