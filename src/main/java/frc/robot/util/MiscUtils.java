package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class MiscUtils {
  public static Pose2d plus(Pose2d a, Translation2d b) {
    return new Pose2d(
        a.getX() + b.getX(), a.getY() + b.getY(), new Rotation2d(a.getRotation().getRadians()));
  }

  public static Optional<Boolean> pullSecondActiveCharacter() {
    Optional<Boolean> ret;
    String gameData;
    gameData = DriverStation.getGameSpecificMessage();

    if (gameData.length() > 0) {
      switch (gameData.charAt(0)) {
        case 'B' :
          ret = Optional.of(false);
          break;
        case 'R' :
          ret = Optional.of(true);
          break;
        default :
          ret = Optional.empty();
          break;
      }
    }
    else {
      ret = Optional.empty();
    }
    return ret;
  }
}
