package frc.robot.subsystems.VisionSubsystem;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;
import java.util.Map;

public class LimelightSubsystem implements VisionBackend {
  private NetworkTableInstance table = null;

  /**
   * Light modes for Limelight.
   *
   * @author Dan Waxman
   */
  public enum LightMode {
    eOn,
    eOff,
    eBlink
  }

  /**
   * Camera modes for Limelight.
   *
   * @author Dan Waxman
   */
  public enum CameraMode {
    eVision,
    eDriver
  }

  public Map<LEDMode, LightMode> ledModeMap =
      Map.of(
          LEDMode.ON, LightMode.eOn,
          LEDMode.OFF, LightMode.eOff,
          LEDMode.BLINK, LightMode.eBlink,
          LEDMode.DEFAULT, LightMode.eOn);

  public Map<VisionBackend.CameraMode, LimelightSubsystem.CameraMode> cameraModeMap =
      Map.of(
          VisionBackend.CameraMode.VISION, LimelightSubsystem.CameraMode.eVision,
          VisionBackend.CameraMode.DRIVER, LimelightSubsystem.CameraMode.eDriver);

  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault();
  }

  @Override
  public boolean hasTargets() {
    return getValue("tv").getDouble(0) == 1;
  }

  @Override
  public double getX() {
    return getValue("tx").getDouble(0.00);
  }

  @Override
  public double getY() {
    return getValue("ty").getDouble(0.00);
  }

  @Override
  public double getA() {
    return getValue("ta").getDouble(0.00);
  }

  @Override
  public void setLEDMode(LEDMode mode) {
    getValue("ledMode").setNumber(ledModeMap.get(mode).ordinal());
  }

  @Override
  public void setCameraMode(frc.robot.subsystems.VisionSubsystem.VisionBackend.CameraMode mode) {
    getValue("camMode").setNumber(cameraModeMap.get(mode).ordinal());
  }

  @Override
  public void setPipeline(int num) {
    getValue("pipeline").setNumber(num);
  }

  @Override
  public double getDistance() {
    if (!hasTargets()) {
      return 0.0;
    }

    double angleToGoalDegrees = LimelightConstants.MOUNTING_ANGLE + getY();
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    double distanceFromLimelightToGoalInches =
        (LimelightConstants.GOAL_HEIGHT - LimelightConstants.LIMELIGHT_HEIGHT)
            / Math.tan(angleToGoalRadians);

    return distanceFromLimelightToGoalInches;
  }

  private NetworkTableEntry getValue(String key) {
    return table.getTable("limelight").getEntry(key);
  }
}
