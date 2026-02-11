package frc.robot.commandGroups;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shoot;
import frc.robot.commands.SwerveCommands.TurnTowardsTarget;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class PointAndShoot extends ParallelCommandGroup {
    public PointAndShoot(CommandSwerveDrivetrain swerveDriveTrain, ShooterSubsystem shooter, HopperSubsystem hopper, BooleanSupplier redside) {
        addCommands(new TurnTowardsTarget(swerveDriveTrain).alongWith(new Shoot(swerveDriveTrain, shooter, hopper, redside)));
    }
}
