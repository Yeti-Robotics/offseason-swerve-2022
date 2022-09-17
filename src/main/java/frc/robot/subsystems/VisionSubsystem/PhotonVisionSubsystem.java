package frc.robot.subsystems.VisionSubsystem;

import frc.robot.Constants.LimelightConstants;
import java.util.Map;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem implements VisionBackend {
  private final PhotonCamera camera;

  public Map<LEDMode, VisionLEDMode> ledModeMap =
      Map.of(
          LEDMode.ON, VisionLEDMode.kOn,
          LEDMode.OFF, VisionLEDMode.kOff,
          LEDMode.DEFAULT, VisionLEDMode.kDefault,
          LEDMode.BLINK, VisionLEDMode.kBlink);

  public Map<CameraMode, Boolean> cameraModeMap =
      Map.of(
          CameraMode.VISION, false,
          CameraMode.DRIVER, true);

  public PhotonVisionSubsystem() {
    camera = new PhotonCamera("yetiworm");
  }

  @Override
  public boolean hasTargets() {
    return camera.getLatestResult().hasTargets();
  }

  @Override
  public double getX() {
    PhotonTrackedTarget latestTarget = camera.getLatestResult().getBestTarget();
    return (latestTarget == null) ? 0.0 : latestTarget.getYaw();
  }

  @Override
  public double getY() {
    PhotonTrackedTarget latestTarget = camera.getLatestResult().getBestTarget();
    return (latestTarget == null) ? 0.0 : latestTarget.getPitch();
  }

  @Override
  public double getA() {
    PhotonTrackedTarget latestTarget = camera.getLatestResult().getBestTarget();
    return (latestTarget == null) ? 0.0 : latestTarget.getArea();
  }

  @Override
  public void setLEDMode(LEDMode mode) {
    camera.setLED(ledModeMap.get(mode));
  }

  @Override
  public void setCameraMode(CameraMode mode) {
    camera.setDriverMode(cameraModeMap.get(mode));
  }

  @Override
  public void setPipeline(int num) {
    camera.setPipelineIndex(num);
  }

  @Override
  public double getDistance() {
    if (!camera.getLatestResult().hasTargets()) {
      return 0.0;
    }

    double angleToGoalDegrees = LimelightConstants.MOUNTING_ANGLE + getY();
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    double distanceFromLimelightToGoalInches =
        (LimelightConstants.GOAL_HEIGHT - LimelightConstants.LIMELIGHT_HEIGHT)
            / Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }
}
