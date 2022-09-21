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
    public static final class OIConstants {
        public static final int DRIVER_STATION_JOY = 0;
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
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(21.75);
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
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

        public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(147.074); // FIXME Measure and set front
                                                                                          // left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 9; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 8; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(255.410); // FIXME Measure and set
                                                                                           // front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 13; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 5; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(143.877); // FIXME Measure and set back
                                                                                         // left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 11; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 10; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(125.859); // FIXME Measure and set back
                                                                                          // right steer offset

        public static final double FALCON_FREE_RPM = 6380.0; // free speed RPM @12V, found here https://motors.vex.com/vexpro-motors/falcon
                                                             // if max voltage changes this value should be modified
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
}
