// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    }

    public static final class ShooterConstants {
        // shooter motor ports
        public static final int SHOOTER_LEFT_FALCON = 8; // left
        public static final int SHOOTER_RIGHT_FALCON = 7; // right

        public static final double SHOOTER_P = 0.00015;
        public static final double SHOOTER_I = 0.0;
        public static final double SHOOTER_D = 0.00001;
        public static final double SHOOTER_F = 0.4;

        public static final double SHOOTER_MAX_VEL = 12242.5; // in native encoder units per 100 ms

        // feed forward values; characterized using meters
        public static final double SHOOTER_KS = 0.65977;
        public static final double SHOOTER_KV = 0.44887;
        public static final double SHOOTER_KA = 0.055666;

        // shooter motor speeds
        public static final double SHOOTER_LOW_SPEED = 0.2; // for low goal shots

        // shooter rpm calc constants
        public static final double PULLEY_RATIO = 48.0 / 36.0; // not completely known
        public static final double ENCODER_TIME_CONVERSION = 600.0; // 100 ms per minute
        public static final double ENCODER_RESOLUTION = 2048.0;
        public static final double QUAD_FACTOR = 4.0; // quadrature encoder factor
        public static final double RPM_TOLERANCE = 10.0;

        public static final double FLYWHEEL_DIAMETER_IN = 4.0; // inches
        public static final double FLYWHEEL_DIAMETER_M = 0.1016; // meters
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
        public static final double LIMELIGHT_HEIGHT = 37.5; // inches
        public static final double GOAL_HEIGHT = 108.0; // inches
        public static final double GRAVITY = 386.09; // inches/ sec ^2
        public static final double MOUNTING_ANGLE = 33.47; // deg
    }

    public static final class DriveConstants {
        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_TRACKWIDTH_METERS = 1.0; // FIXME Measure and set trackwidth
        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * Should be measured from center to center.
         */
        public static final double DRIVETRAIN_WHEELBASE_METERS = 1.0; // FIXME Measure and set wheelbase

        public static final int DRIVETRAIN_PIGEON_ID = 0; // FIXME Set Pigeon ID

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 0; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set front
                                                                                          // left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 0; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set
                                                                                           // front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 0; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 0; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back
                                                                                         // left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 0; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 0; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 0; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(0.0); // FIXME Measure and set back
                                                                                          // right steer offset

        public static final double FALCON_FREE_RPM = 6380.0; // free speed RPM @12V, found here
                                                             // https://motors.vex.com/vexpro-motors/falcon
                                                             // if max voltage changes this value should be modified
    }
}
