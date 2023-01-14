// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    /**
     * Creates a new IntakeSubsystem.
     */

    private final CANSparkMax intakeSpark;

    private final DoubleSolenoid intakePistons;

    public IntakeSubsystem() {
        intakePistons = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.INTAKE_PISTONS_SOLENOID[0], IntakeConstants.INTAKE_PISTONS_SOLENOID[1]);
        intakeSpark = new CANSparkMax(IntakeConstants.INTAKE_SPARK, MotorType.kBrushless);

        intakeSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);

        intakeSpark.setInverted(false);

        intakeSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 250);
        intakeSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 250);
        intakeSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 250);
        intakeSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 250);

        intakeSpark.setSmartCurrentLimit(40);
        intakeSpark.enableVoltageCompensation(Constants.MOTOR_VOLTAGE_COMP);

        intakePistons.set(Value.kForward);
    }

    public void extendIntake() {
        intakePistons.set(Value.kReverse);
    }

    public void retractIntake() {
        intakePistons.set(Value.kForward);
    }

    public void rollIn() {
        intakeSpark.set(IntakeConstants.INTAKE_SPEED);
    }

    public void rollIn(double speed) {
        intakeSpark.set(speed);
    }

    public void rollOut() {
        intakeSpark.set(IntakeConstants.INTAKE_OUT_SPEED);
    }

    public void stopRoll() {
        intakeSpark.set(0);
    }

    public Value intakePosition() {
        return intakePistons.get();
    }

    public void toggleIntake() {
        intakePistons.toggle();
    }
}
