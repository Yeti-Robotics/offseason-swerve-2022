package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DriveConstants;

public class SwerveModule {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;

    private final CANCoder absoluteEncoder;
    private final PIDController drivePIDController =
        new PIDController(
            DriveConstants.DRIVE_MOTOR_P,
            DriveConstants.DRIVE_MOTOR_I,
            DriveConstants.DRIVE_MOTOR_D
        );
    private final ProfiledPIDController steeringPIDController =
        new ProfiledPIDController(
            DriveConstants.STEER_MOTOR_P,
            DriveConstants.STEER_MOTOR_I,
            DriveConstants.STEER_MOTOR_D,
            new TrapezoidProfile.Constraints(
                3 * Math.PI,    //540,
                6 * Math.PI) //1080)
        );
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        DriveConstants.DRIVE_MOTOR_KS, DriveConstants.DRIVE_MOTOR_KV, DriveConstants.DRIVE_MOTOR_KA
    );
    private final SimpleMotorFeedforward steerFeedforward = new SimpleMotorFeedforward(
        DriveConstants.STEER_MOTOR_KS, DriveConstants.STEER_MOTOR_KV, DriveConstants.STEER_MOTOR_KA
    );
    private SwerveModuleState state = new SwerveModuleState();

    public SwerveModule(
        int driveMotorID,
        boolean driveInverted,
        int steerMotorID,
        int absoluteEncoderID,
        boolean absoluteEncoderReversed,
        double absoluteEncoderOffsetDeg) {

        absoluteEncoder = new CANCoder(absoluteEncoderID);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        absoluteEncoder.configMagnetOffset(absoluteEncoderOffsetDeg);
        absoluteEncoder.configSensorDirection(absoluteEncoderReversed);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
        absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 250);

        driveMotor = new WPI_TalonFX(driveMotorID);
        steerMotor = new WPI_TalonFX(steerMotorID);

        driveMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(driveInverted);
        steerMotor.setInverted(true);

        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
        driveMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));
        steerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.1));
        steerMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 65, 0.1));

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
        steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        driveMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        driveMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);

        steerMotor.setStatusFramePeriod(StatusFrame.Status_1_General, 250);
        steerMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 250);

        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // resetEncoders();
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);

        double absolutePosition = absoluteEncoder.getAbsolutePosition() * DriveConstants.DEGREES_TO_FALCON;
        steerMotor.setSelectedSensorPosition(0);
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getSteerPosition() {
        return Math.toRadians(absoluteEncoder.getAbsolutePosition());
        // return steerMotor.getSelectedSensorPosition() / DriveConstants.DEGREES_TO_FALCON;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * 10 / 2048
            * SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    }

    public double getSteerVelocity() {
        return steerMotor.getSelectedSensorVelocity() * 10;
    }

    public SwerveModuleState getState() {
        // updateState();
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerPosition()));
    }

    public void updateState() {
        state.speedMetersPerSecond = getDriveVelocity();
        state.angle = new Rotation2d(getSteerPosition());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        double driveVelocity = getDriveVelocity();
        double steerAngle = getSteerPosition();

        if (Math.abs(desiredState.speedMetersPerSecond) < 0.01
            && Math.abs(desiredState.angle.getRadians() - steerAngle) < 0.05) {
            stop();
            return;
        }
        desiredState = SwerveModuleState.optimize(desiredState, new Rotation2d(steerAngle));

        final double driveOutput =
            drivePIDController.calculate(driveVelocity, desiredState.speedMetersPerSecond)
                + driveFeedforward.calculate(desiredState.speedMetersPerSecond);

        final double steerOutput =
            steeringPIDController.calculate(steerAngle, desiredState.angle.getRadians())
                + steerFeedforward.calculate(steeringPIDController.getSetpoint().velocity);

        driveMotor.setVoltage(desiredState.speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
            * DriveConstants.MAX_VOLTAGE);
        // driveMotor.setVoltage(driveOutput);
        // steerMotor.set(steeringPIDController.calculate(getSteerPosition(), desiredState.angle.getDegrees()));
        // steerMotor.set(steeringPIDController.calculate(getSteerPosition(), 45));
        steerMotor.setVoltage(steerOutput);
//
//        steerMotor.set(ControlMode.Position, desiredState.angle.getDegrees() * DriveConstants.DEGREES_TO_FALCON);
    }

    public void stop() {
        driveMotor.setVoltage(0.0);
        steerMotor.set(0.0);
    }
}
