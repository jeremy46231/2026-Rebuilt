package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class MoveArmToAngle extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final ArmSubsystem armSubsystem;

  public MoveArmToAngle(ArmSubsystem armSubsystem) {
    this.armSubsystem = armSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setPosition(20);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
