package frc.robot.subsystems.VisionSubsystem;

public interface VisionBackend {

    public boolean hasTargets();

    public double getX();

    public double getY();

    public double getA();

    public void setLEDMode(LEDMode mode);

    public void setCameraMode(CameraMode mode);

    public void setPipeline(int num);

    public double getDistance();

    public enum LEDMode {
        ON,
        OFF,
        DEFAULT,
        BLINK
    }

    public enum CameraMode {
        VISION,
        DRIVER
    }
}
