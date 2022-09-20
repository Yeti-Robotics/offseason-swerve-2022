package frc.robot;

    // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.GeomUtil;

    /**
     * Contains various field dimensions and useful side points. All dimensions are in meters.
     * Length of the field is the x-axis and the width is the y-axis.
     * Positive x is towards the opposing alliance, positive y is to the left of the driver.
     * (0, 0) is the center of the hub.
     * Assume all angles reference 0 degrees facing the opposing alliance.
     */
public final class FieldConstants {

        // Field dimensions
        public static final double fieldLength = Units.inchesToMeters(54.0 * 12.0);
        public static final double fieldWidth = Units.inchesToMeters(27.0 * 12.0);
        public static final double hangarLength = Units.inchesToMeters(128.75);
        public static final double hangarWidth = Units.inchesToMeters(116.0);

        // Vision target
        public static final double visionTargetDiameter =
                Units.inchesToMeters(4.0 * 12.0 + 5.375);
        public static final double visionTargetHeightLower =
                Units.inchesToMeters(8.0 * 12.0 + 5.625); // Bottom of tape
        public static final double visionTargetHeightUpper =
                visionTargetHeightLower + Units.inchesToMeters(2.0); // Top of tape

        // Dimensions of hub and tarmac
        public static final Rotation2d centerLineAngleCCW = Rotation2d.fromDegrees(114.0);
        public static final Rotation2d centerLineAngleCW = Rotation2d.fromDegrees(-66.0);
        public static final Translation2d hubCenter =
                new Translation2d(0, 0);

        /**
         * Diameter of a circle tangent to each side of the octagon
         */
        public static final double tarmacInnerDiameter = Units.inchesToMeters(219.25);
        /**
         * Diameter of a circle tangent to each vertex of the octagon
         */
        public static final double tarmacOuterDiameter = Units.inchesToMeters(237.31);
        public static final double tarmacFenderToTip = Units.inchesToMeters(84.75);
        public static final double tarmacFullSideLength =
                tarmacInnerDiameter * (Math.sqrt(2.0) - 1.0); // If the tarmac formed a full octagon
        public static final double tarmacMarkedSideLength =
                Units.inchesToMeters(82.83); // Length of tape marking outside of tarmac
        public static final double gapBetweenTarmacsLength = Units.inchesToMeters(14.75);
        public static final double tarmacMissingSideLength =
                tarmacFullSideLength - tarmacMarkedSideLength; // Length removed b/c of corner cutoff
        public static final double hubSquareLength =
                tarmacOuterDiameter - (tarmacFenderToTip * 2.0);
        private static final Translation2d octagonVertexNoRotation = hubCenter.minus(
                new Translation2d(tarmacOuterDiameter/2, 0));
        public static final Rotation2d octagonVertexOffset = Rotation2d.fromDegrees(66.0-45.0);


        /*
        *   From the perspective of the driver.
        *   Left tarmac is tarmac A and the right tarmac is tarmac B
        *   Letters are assigned from the left most item starting with A
        *   For example, the left most vertex of the tarmac is tarmacVertex A;
        *   the next vertex to the right is tarmacVertexB, so on and so forth.
        */
        public static final class TarmacPositions {
                // X, Y coordinates of the vertexes of the tarmacs.
                public static final Translation2d tarmacVertexA = octagonVertexNoRotation.plus(new Translation2d(
                                Math.tan(Math.toRadians(135.0 / 2.0)) / (gapBetweenTarmacsLength / 2), -gapBetweenTarmacsLength / 2))
                        .rotateBy(centerLineAngleCW);
                public static final Translation2d tarmacVertexB =
                        octagonVertexNoRotation.rotateBy(octagonVertexOffset.unaryMinus());
                public static final Translation2d tarmacVertexC = octagonVertexNoRotation.plus(new Translation2d(
                                Math.tan(Math.toRadians(135.0 / 2.0)) / (gapBetweenTarmacsLength / 2), gapBetweenTarmacsLength / 2))
                        .rotateBy(octagonVertexOffset);
                public static final Translation2d tarmacVertexD = tarmacVertexA.rotateBy(Rotation2d.fromDegrees(90));
                public static final Translation2d tarmacVertexE = tarmacVertexB.rotateBy(Rotation2d.fromDegrees(90));
                public static final Translation2d tarmacVertexF = tarmacVertexC.rotateBy(Rotation2d.fromDegrees(90));

                public static final Rotation2d tarmacAngleA = Rotation2d.fromDegrees(135.0);
                public static final Rotation2d tarmacAngleB = Rotation2d.fromDegrees(180.0);
                public static final Rotation2d tarmacAngleC = Rotation2d.fromDegrees(225.0);
                public static final Rotation2d tarmacAngleD = Rotation2d.fromDegrees(270.0);
        }

        public static final class CargoPositions {
                // X, Y coordinates of the cargo
                public static final double cargoCircleRadius = Units.inchesToMeters(153.0);
                public static final Rotation2d cargoAngle = Rotation2d.fromDegrees(360.0 / 16.0);
                public static final Translation2d cargoA = hubCenter.plus(
                                new Translation2d(cargoCircleRadius, 0)).
                        rotateBy(centerLineAngleCCW.plus(Rotation2d.fromDegrees(360.0 / 32.0)));
                public static final Translation2d cargoB = cargoA.rotateBy(cargoAngle);
                public static final Translation2d cargoC = cargoB.rotateBy(cargoAngle.plus(cargoAngle));
                public static final Translation2d cargoD = cargoC.rotateBy(cargoAngle);
                public static final Translation2d cargoE = cargoD.rotateBy(cargoAngle.plus(cargoAngle));
                public static final Translation2d cargoF = cargoE.rotateBy(cargoAngle);
                public static final Translation2d terminalCargo = new Translation2d(Units.inchesToMeters(282.083), Units.inchesToMeters(117.82));
        }

        // Terminal cargo point
        public static final Rotation2d terminalOuterRotation =
                Rotation2d.fromDegrees(133.75);
        public static final double terminalLength =
                Units.inchesToMeters(324.0 - 256.42);
        public static final double terminalWidth = Math.tan(
                Rotation2d.fromDegrees(180.0).minus(terminalOuterRotation).getRadians())
                * terminalLength;
        public static final Pose2d terminalCenter =
                new Pose2d(new Translation2d(terminalLength / 2.0, terminalWidth / 2.0),
                        terminalOuterRotation.minus(Rotation2d.fromDegrees(90.0)));
        public static final double terminalCargoOffset = Units.inchesToMeters(10.43);
        public static final Pose2d cargoG = terminalCenter
                .transformBy(GeomUtil.transformFromTranslation(terminalCargoOffset, 0.0));

        // Opposite reference points
        public static final Translation2d opposieTarmacVertexA = TarmacPositions.tarmacVertexA.unaryMinus();
        public static final Translation2d opposieTarmacVertexB = TarmacPositions.tarmacVertexB.unaryMinus();
        public static final Translation2d opposieTarmacVertexC = TarmacPositions.tarmacVertexC.unaryMinus();
        public static final Translation2d opposieTarmacVertexD = TarmacPositions.tarmacVertexD.unaryMinus();
        public static final Translation2d opposieTarmacVertexE = TarmacPositions.tarmacVertexE.unaryMinus();
        public static final Translation2d opposieTarmacVertexF = TarmacPositions.tarmacVertexF.unaryMinus();

        // Opposite cargo points
        public static final Translation2d oppositeCargoA = CargoPositions.cargoA.unaryMinus();
        public static final Translation2d oppositeCargoB = CargoPositions.cargoB.unaryMinus();
        public static final Translation2d oppositeCargoC = CargoPositions.cargoC.unaryMinus();
        public static final Translation2d oppositeCargoD = CargoPositions.cargoD.unaryMinus();
        public static final Translation2d oppositeCargoE = CargoPositions.cargoE.unaryMinus();
        public static final Translation2d oppositeCargoF = CargoPositions.cargoF.unaryMinus();
        public static final Translation2d oppositeTerminalCargo = CargoPositions.terminalCargo.unaryMinus();
    }

