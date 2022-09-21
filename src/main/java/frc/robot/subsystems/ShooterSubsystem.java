package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonFX shooterLeftFalcon;
    private final WPI_TalonFX shooterRightFalcon;

    private final MotorControllerGroup shooterFalcons;

    public enum ShooterMode {
        LIMELIGHT,
        MANUAL,
        LOWGOAL,
        OFF
    }

    private ShooterMode shooterMode;

    private double setPoint = 0.0;
    public static boolean atSetPoint = false;

    private final PIDController shooterPID;
    private final SimpleMotorFeedforward feedForward;

    public ShooterSubsystem() {
        shooterLeftFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_LEFT_FALCON);
        shooterRightFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_RIGHT_FALCON);

        shooterFalcons = new MotorControllerGroup(shooterLeftFalcon, shooterRightFalcon);

        shooterLeftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        shooterRightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        shooterLeftFalcon.follow(shooterRightFalcon);
        shooterLeftFalcon.setInverted(InvertType.OpposeMaster);

        shooterMode = ShooterMode.OFF;

        shooterLeftFalcon.setNeutralMode(NeutralMode.Coast);
        shooterRightFalcon.setNeutralMode(NeutralMode.Coast);

        shooterLeftFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);
        shooterRightFalcon.configVoltageCompSaturation(Constants.MOTOR_VOLTAGE_COMP);

        shooterLeftFalcon.enableVoltageCompensation(true);
        shooterRightFalcon.enableVoltageCompensation(true);

        shooterPID = new PIDController(
                ShooterConstants.SHOOTER_P, ShooterConstants.SHOOTER_I, ShooterConstants.SHOOTER_D);
        feedForward = new SimpleMotorFeedforward(
                ShooterConstants.SHOOTER_KS, ShooterConstants.SHOOTER_KV, ShooterConstants.SHOOTER_KA);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Set Point: ", setPoint);
        SmartDashboard.putNumber("Flywheel Voltage", shooterRightFalcon.getMotorOutputVoltage());
        atSetPoint = setPoint >= getMetersPerSecond() - ShooterConstants.VELOCITY_TOLERANCE
                && shooterMode != ShooterMode.OFF;

            switch (shooterMode) {
                case LIMELIGHT:
                    if (VisionSubsystem.getDistance() == 0.0) {
                        setPoint = 12;
                        shootFlywheel(setPoint);
                        break;
                    }
                    setPoint = 25.0/3.0 * VisionSubsystem.getDistance() + 2991.66667;
                    shootFlywheel(
                            feedForward.calculate(setPoint, 10.0)
                                    + shooterPID.calculate(getMetersPerSecond(), setPoint));
                    break;
                case MANUAL:
                    shootFlywheel(
                            feedForward.calculate(setPoint, 10.0)
                                    + shooterPID.calculate(getMetersPerSecond(), setPoint));
                    break;
                case LOWGOAL:
                    shootFlywheel(ShooterConstants.SHOOTER_LOW_SPEED);
                    break;
                default:
                    stopFlywheel();
                    break;
            }
    }

    /**
     *
     * @param setPoint in meters/second
     */
    public void setSetPoint(double setPoint) {
        if (setPoint > ShooterConstants.MAX_VELOCITY) {
            this.setPoint = 32.0;
            return;
        }
        this.setPoint = setPoint;
    }

    public void setShooterMode(ShooterMode shooterMode) {
        this.shooterMode = shooterMode;
    }

    public ShooterMode getShooterMode() {
        return this.shooterMode;
    }

    private void shootFlywheel(double speed) {
        shooterFalcons.set(speed);
    }

    private void setFlywheelVolts(double volts) {
        shooterFalcons.setVoltage(volts);
    }

    public void stopFlywheel() {
        shooterFalcons.stopMotor();
        setPoint = 0.0;
        shooterMode = ShooterMode.OFF;
    }

    public double getLeftEncoder() {
        return shooterLeftFalcon.getSelectedSensorVelocity();
    }

    public double getRightEncoder() {
        return shooterRightFalcon.getSelectedSensorVelocity();
    }

    public double getAverageEncoder() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public double getFlywheelRPM() {
        return getAverageEncoder() * ShooterConstants.ENCODER_TIME_CONVERSION
                / ShooterConstants.ENCODER_RESOLUTION
                * ShooterConstants.PULLEY_RATIO;
    }

    public double getMetersPerSecond() {
        return (ShooterConstants.FLYWHEEL_DIAMETER_M * Math.PI) * (getFlywheelRPM() / 60.0);
    }

    public double getVelocityUnitsFromRPM() {
        return getFlywheelRPM() / ShooterConstants.PULLEY_RATIO
                * ShooterConstants.ENCODER_TIME_CONVERSION
                / ShooterConstants.ENCODER_RESOLUTION;
    }

    // returns in volts
    public double getFeedForward() {
        return (Constants.MOTOR_VOLTAGE_COMP / 8750.0) * setPoint;
    }
}