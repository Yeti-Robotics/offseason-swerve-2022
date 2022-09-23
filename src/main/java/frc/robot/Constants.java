// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double MOTOR_VOLTAGE_COMP = 11.0;

    public static final class OIConstants {
        public static final int DRIVER_STATION_JOY = 0;

        public static final double DEADBAND = 0.2;
    }

    public static final class DriveConstants {
        public static final Transform2d X_ROBOT_CENTER = new Transform2d(
                new Translation2d(Units.inchesToMeters(33.5215/2.0), 0.0),
                Rotation2d.fromDegrees(0.0));
        public static final Transform2d Y_ROBOT_CENTER = new Transform2d(
                new Translation2d(0.0, Units.inchesToMeters(33.5215/2.0)),
                Rotation2d.fromDegrees(0.0));
        public static final Transform2d ROBOT_CENTER = new Transform2d(
                new Translation2d(Units.inchesToMeters(33.5215/2.0), Units.inchesToMeters(33.5215/2.0)),
                Rotation2d.fromDegrees(0.0));
        /**
         * The left-to-right distance between the drivetrain wheels
         * <p>
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21.75);
        /**
         * The front-to-back distance between the drivetrain wheels.
         * <p>
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(21.75);

        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         * The formula for calculating the theoretical maximum velocity is:
         * [Motor free speed RPM] / 60 * [Drive reduction] * [Wheel diameter meters] *
         * pi
         */
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                        DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

        public static final double MAX_ACCELERATION = MAX_VELOCITY_METERS_PER_SECOND * 0.75;
        public static final double MAX_ANGULAR_ACCELERATION = MAX_VELOCITY_METERS_PER_SECOND * 0.75;

        public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(152.666); // FIXME Measure and set front
                                                                                          // left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(260.859); // FIXME Measure and set
                                                                                           // front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 13; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 5; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(149.502); //FIXME Measure and set back
                                                                                         // left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(131.); // FIXME Measure and set back
        // right steer offset

        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;
        public static final double FALCON_FREE_RPM = 6380.0; // free speed RPM @12V, found here https://motors.vex.com/vexpro-motors/falcon
        // if max voltage changes this value should be modified

        public static final double STEER_MOTOR_P = 0.1; //0.08 //0.064
        public static final double STEER_MOTOR_I = 0.0;
        public static final double STEER_MOTOR_D = 0.0;
        public static final double STEER_MOTOR_F = 0.0;

        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
        public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
        public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
    }

    public static final class AutoConstants {
        /**
         * Max velocity in meters per second
         */
        public static final double MAX_VELOCITY = DriveConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.75;
        /**
         * Max acceleration in meters per second squared
         */
        public static final double MAX_ACCEL = DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.75;

        public static final double X_CONTROLLER_P = 0.0;
        public static final double Y_CONTROLLER_P = 0.0;
        public static final double THETA_CONTROLLER_P = 0.0;
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = //
                new TrapezoidProfile.Constraints(
                        DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }

    public static final class ShooterConstants {
        // shooter motor ports
        public static final int SHOOTER_LEFT_FALCON = 14; // left
        public static final int SHOOTER_RIGHT_FALCON = 15; // right

        public static final double SHOOTER_P = 0.00015;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.00001;
        public static final double SHOOTER_F = 0.4;

        public static final double TARGETING_P = 0.02;
        public static final double TARGETING_I = 0.00;
        public static final double TARGETING_D = 0.00;

        public static final double TARGET_OFFSET = Units.inchesToMeters(8.0);


        public static final double MAX_VELOCITY = 32.0; // in meters/second

        // feed forward values; characterized using meters
        public static final double SHOOTER_KS = 0.60;
        public static final double SHOOTER_KV = 0.27; // Volts * second/meter
        public static final double SHOOTER_KA = 0.07; // Volts * second^2/meter

        // shooter motor speeds
        public static final double SHOOTER_LOW_SPEED = 0.2; // for low goal shots

        // shooter rpm calc constants
        public static final double PULLEY_RATIO = 48.0 / 36.0;
        public static final double ENCODER_TIME_CONVERSION = 600.0; // 100 ms per minute
        public static final double ENCODER_RESOLUTION = 2048.0;
        public static final double QUAD_FACTOR = 4.0; // quadrature encoder factor
        public static final double VELOCITY_TOLERANCE = 0.5;

        public static final double FLYWHEEL_DIAMETER_IN = 4.0; // inches
        public static final double FLYWHEEL_DIAMETER_M = Units.inchesToMeters(FLYWHEEL_DIAMETER_IN); // meters
        public static final double SHOOTER_HIGH_DIST = 82.0; // inches; ideal dist from shooter to go high
        public static final double SHOOTER_DIST_TOLERANCE = 6.0; // inhces; see above
    }

    public static final class LimelightConstants {
        // distance calc constants
        public static final double KNOWN_DISTANCE = 161.3; // inches
        public static final int PIXEL_WIDTH_KNOWN = 65; // pixels
        public static final double KNOWN_TAPE_BOUND_WIDTH = 39.25; // inches
        public static final double FOCAL_LENGTH = (KNOWN_DISTANCE * PIXEL_WIDTH_KNOWN) / KNOWN_TAPE_BOUND_WIDTH;

        // trajectory constants
        public static final double LIMELIGHT_HEIGHT = Units.inchesToMeters(28.4); // meters
        public static final double GOAL_HEIGHT = Units.inchesToMeters(108.0); // meters
        public static final double GRAVITY = 386.09; // inches/ sec ^2
        public static final double MOUNTING_ANGLE = 31.5; // deg
    }

    public static final class IntakeConstants {
        public static final int INTAKE_SPARK = 18;
        public static final int[] INTAKE_PISTONS_SOLENOID = {0,1};
        public static final double INTAKE_SPEED = 0.3;
        public static final double INTAKE_OUT_SPEED = 0.3;
    }

    public static final class NeckConstants {
        public static final int FRONT_INDEXER = 17;
        public static final int BACK_INDEXER = 16;
        public static final int LOW_BEAM_BREAK = 6;
        public static final int HIGH_BEAM_BREAK = 7;
        public static final double NECK_FRONT_SPEED = 0.2;
        public static final double NECK_BACK_SPEED = 0.2;
        public static final double NECK_FRONT_OUT_SPEED = 0.2;
        public static final double NECK_BACK_OUT_SPEED = 0.2;
    }
}
