package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem.VisionSubsystem;
import frc.robot.utils.MoveAndShootController;

public class ShooterSubsystem extends SubsystemBase {
    private final WPI_TalonFX shooterLeftFalcon;
    private final WPI_TalonFX shooterRightFalcon;

    private final MotorControllerGroup shooterFalcons;

    public enum ShooterMode {
        TEST_MOVE,
        LIMELIGHT,
        MANUAL,
        LOWGOAL,
        OFF
    }

    private static ShooterMode shooterMode;

    private double setPoint = 0.0;
    private double acceleration = 0.0;
    public static boolean atSetPoint = false;

    private final PIDController shooterPID;
    private final SimpleMotorFeedforward feedForward;
    private final MoveAndShootController moveAndShootController;

    public ShooterSubsystem(DrivetrainSubsystem drivetrainSubsystem, MoveAndShootController moveAndShootController) {
        shooterLeftFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_LEFT_FALCON);
        shooterRightFalcon = new WPI_TalonFX(ShooterConstants.SHOOTER_RIGHT_FALCON);

        shooterFalcons = new MotorControllerGroup(shooterLeftFalcon, shooterRightFalcon);

        shooterLeftFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        shooterRightFalcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        shooterLeftFalcon.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        shooterLeftFalcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        shooterRightFalcon.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        shooterRightFalcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

        shooterRightFalcon.setInverted(true);
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
        this.moveAndShootController = moveAndShootController;
    }

    @Override
    public void periodic() {
        atSetPoint = setPoint >= getMetersPerSecond() - ShooterConstants.VELOCITY_TOLERANCE
                && shooterMode != ShooterMode.OFF;

            switch (shooterMode) {
                case TEST_MOVE:
                    if (VisionSubsystem.getDistance() == 0.0) {
                        setPoint = 12;
                        shootFlywheel(setPoint);
                        break;
                    }
                    setSetPoint(0.1389/0.9144 * VisionSubsystem.getDistance() + 15.9150 + moveAndShootController.calculateShooterSpeed());
                    setFlywheelVolts(
                            feedForward.calculate(setPoint, acceleration)
                                    + shooterPID.calculate(getMetersPerSecond(), setPoint));
                    break;
                case LIMELIGHT:
                    if (VisionSubsystem.getDistance() == 0.0) {
                        setPoint = 12;
                        shootFlywheel(setPoint);
                        break;
                    }
                    setSetPoint(0.1389/0.9144 * VisionSubsystem.getDistance() + 15.9150);
                    setFlywheelVolts(
                            feedForward.calculate(setPoint, acceleration)
                                    + shooterPID.calculate(getMetersPerSecond(), setPoint));
                    break;
                case MANUAL:
                    setFlywheelVolts(
                            feedForward.calculate(setPoint, acceleration)
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
        this.acceleration = setPoint > 21.0 ? 17.0 : 0.8 * setPoint;
        this.setPoint = setPoint > ShooterConstants.MAX_VELOCITY ? 32 : setPoint;
    }

    public void setShooterMode(ShooterMode shooterMode) {
        ShooterSubsystem.shooterMode = shooterMode;
    }

    public static ShooterMode getShooterMode() {
        return shooterMode;
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
        return getLeftEncoder() * ShooterConstants.ENCODER_TIME_CONVERSION
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