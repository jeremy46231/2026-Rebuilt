package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision.fuelGauge;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetection extends SubsystemBase {

  private static ObjectDetection[] cameraList;

  private final Constants.Vision.Cameras cameraID;
  private String cameraTitle;

  private final PhotonCamera photonCamera;
  private PhotonPipelineResult latestVisionResult;

  public ObjectDetection(Constants.Vision.Cameras cameraID) {
    this.cameraID = cameraID;
    photonCamera = new PhotonCamera(cameraID.toString());

    cameraTitle = cameraID.getLoggingName();
  }

  public static ObjectDetection getInstance(Constants.Vision.Cameras cameraID) {
    int index = cameraID.ordinal();
    if (cameraList[index] == null) cameraList[index] = new ObjectDetection(cameraID);
    return cameraList[index];
  }

  public void periodic() {

    List<PhotonPipelineResult> results = photonCamera.getAllUnreadResults();
    for (var result : results) latestVisionResult = result;

    Optional<PhotonTrackedTarget> blob = getLargestBlob();
    blob.ifPresentOrElse(
        b -> {
          DogLog.log("Vision/BlobPresent", true);
          DogLog.log("Vision/BlobYaw", b.getYaw());
          DogLog.log("Vision/BlobPitch", b.getPitch());
          DogLog.log("Vision/BlobSkew", b.getSkew());
          double maxFuelPercentage =
              ((double)
                      Math.round(
                          b.getArea()
                              / Constants.Vision.MAX_DETECTABLE_FUEL_AREA_PERCENTAGE
                              * 100.0
                              / 10.0))
                  * 10.0;

          double maxFuelRealisticPercentage =
              ((double)
                      Math.round(
                          b.getArea()
                              / Constants.Vision.REALISTIC_MAX_DETECTABLE_AREA_PERCENTAGE
                              * 100.0
                              / 10.0))
                  * 10.0;

          DogLog.log("Vision/FuelGauge", maxFuelPercentage);
          DogLog.log("Vision/FuelGaugeRealistic", maxFuelRealisticPercentage);

          logThresholdStates(maxFuelPercentage, maxFuelRealisticPercentage);
        },
        () -> DogLog.log("Vision/BlobPresent", false));
  }

  public void logThresholdStates(double max, double maxRealistic) {
    fuelGauge gauge, realisticGauge;

    if (max < fuelGauge.EMPTY.getThreshold()) {
      gauge = fuelGauge.EMPTY;
    } else if (max < fuelGauge.LOW.getThreshold()) {
      gauge = fuelGauge.LOW;
    } else if (max < fuelGauge.MEDIUM.getThreshold()) {
      gauge = fuelGauge.MEDIUM;
    } else {
      gauge = fuelGauge.FULL;
    }

    if (maxRealistic < fuelGauge.EMPTY.getThreshold()) {
      realisticGauge = fuelGauge.EMPTY;
    } else if (maxRealistic < fuelGauge.LOW.getThreshold()) {
      realisticGauge = fuelGauge.LOW;
    } else if (maxRealistic < fuelGauge.MEDIUM.getThreshold()) {
      realisticGauge = fuelGauge.MEDIUM;
    } else {
      realisticGauge = fuelGauge.FULL;
    }

    DogLog.log("Vision/FuelGaugeLevel", gauge.toString());
    DogLog.log("Vision/FuelGaugeRealisticLevel", realisticGauge.toString());
  }

  public Optional<PhotonTrackedTarget> getLargestBlob() {
    if (latestVisionResult == null) return Optional.empty();
    List<PhotonTrackedTarget> targets = latestVisionResult.getTargets();
    if (targets.isEmpty()) return Optional.empty();

    return targets.stream().max((a, b) -> Double.compare(a.getArea(), b.getArea()));
  }
}
