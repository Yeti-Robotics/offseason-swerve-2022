package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.*;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.CTREModuleState;

public class SwerveModule {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steerMotor;

    private final WPI_CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;

    private final PIDController steeringPIDController;

    public SwerveModule(
            int driveMotorID, int steerMotorID,
            boolean driveInverted,
            int absoluteEncoderID, boolean absoluteEncoderReversed, double absoluteEncoderOffsetRad) {

        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new WPI_CANCoder(absoluteEncoderID);
        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configAllSettings(getEncoderSettings());
        absoluteEncoder.setPosition(absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad);
        absoluteEncoder.configMagnetOffset(Math.toDegrees(absoluteEncoderOffsetRad));

        driveMotor = new WPI_TalonFX(driveMotorID);
        steerMotor = new WPI_TalonFX(steerMotorID);
        steerMotor.configFactoryDefault();
        steerMotor.configAllSettings(getSteerMotorSettings());

        driveMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        driveMotor.setInverted(driveInverted);

//        steerMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 15, 0.5));

        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);
//        steerMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

        steeringPIDController = new PIDController(DriveConstants.STEER_MOTOR_P, 0.0, DriveConstants.STEER_MOTOR_D);
        steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    private CANCoderConfiguration getEncoderSettings() {
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.sensorDirection = this.absoluteEncoderReversed;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.unitString = "rad";
        return config;
    }

    private TalonFXConfiguration getSteerMotorSettings() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = DriveConstants.STEER_MOTOR_P;
        config.slot0.kI = DriveConstants.STEER_MOTOR_I;
        config.slot0.kD = DriveConstants.STEER_MOTOR_D;
        config.slot0.kF = DriveConstants.STEER_MOTOR_F;
        config.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                DriveConstants.ANGLE_ENABLE_CURRENT_LIMIT,
                DriveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT,
                DriveConstants.ANGLE_PEAK_CURRENT_LIMIT,
                DriveConstants.ANGLE_PEAK_CURRENT_DURATION
        );
        config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        return config;
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        steerMotor.setSelectedSensorPosition(0);
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    public double getSteerRad() {
        return absoluteEncoder.getAbsolutePosition();
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
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getSteerRad()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
//        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        driveMotor.setVoltage(desiredState.speedMetersPerSecond / DriveConstants.MAX_VELOCITY_METERS_PER_SECOND
         * DriveConstants.MAX_VOLTAGE);
        steerMotor.set(steeringPIDController.calculate(getSteerRad(), desiredState.angle.getRadians()));
    }

    public void stop() {
        driveMotor.setVoltage(0.0);
        steerMotor.set(0.0);
    }
}
