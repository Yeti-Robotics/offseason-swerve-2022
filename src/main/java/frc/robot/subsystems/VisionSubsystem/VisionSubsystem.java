package frc.robot.subsystems.VisionSubsystem;

import frc.robot.subsystems.VisionSubsystem.VisionBackend.CameraMode;
import frc.robot.subsystems.VisionSubsystem.VisionBackend.LEDMode;

public class VisionSubsystem {
  private static final VisionAPI api = VisionAPI.PHOTONVISION;
  private static final VisionBackend visionSubsystem = getBackend();
  // change this to change what backend we use

  public enum VisionAPI {
    LIMELIGHT,
    PHOTONVISION
  }

  public static VisionBackend getBackend() {
    switch (api) {
      case PHOTONVISION:
        return new PhotonVisionSubsystem();
      case LIMELIGHT:
        return new LimelightSubsystem();
      default:
        // hopefully never happens
        return null;
    }
  }

  public static boolean hasTargets() {
    return visionSubsystem.hasTargets();
  }

  public static double getX() {
    return visionSubsystem.getX();
  }

  public static double getY() {
    return visionSubsystem.getY();
  }

  public static double getA() {
    return visionSubsystem.getA();
  }

  public static void setLEDMode(LEDMode mode) {
    visionSubsystem.setLEDMode(mode);
  }

  public static void setCameraMode(CameraMode mode) {
    visionSubsystem.setCameraMode(mode);
  }

  public static void setPipeline(int num) {
    visionSubsystem.setPipeline(num);
  }

  public static double getDistance() {
    return visionSubsystem.getDistance();
  }
}
