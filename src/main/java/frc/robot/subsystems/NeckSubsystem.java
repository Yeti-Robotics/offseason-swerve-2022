package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class NeckSubsystem extends SubsystemBase {
    private final CANSparkMax frontSpark;
    private final CANSparkMax backSpark;
    private final DigitalInput lowBeamBreak;
    private final DigitalInput highBeamBreak;

    public NeckSubsystem() {
        frontSpark = new CANSparkMax(Constants.NeckConstants.FRONT_INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);
        backSpark = new CANSparkMax(Constants.NeckConstants.BACK_INDEXER, CANSparkMaxLowLevel.MotorType.kBrushless);

        frontSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        backSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);

        frontSpark.setInverted(false);
        backSpark.setInverted(false);

        frontSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 250);
        frontSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 250);
        frontSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 250);
        frontSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 250);

        backSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus0, 250);
        backSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus1, 250);
        backSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 250);
        backSpark.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus3, 250);

        frontSpark.setSmartCurrentLimit(20);
        backSpark.setSmartCurrentLimit(20);

        lowBeamBreak = new DigitalInput(Constants.NeckConstants.LOW_BEAM_BREAK);
        highBeamBreak = new DigitalInput(Constants.NeckConstants.HIGH_BEAM_BREAK);
    }

    public void moveUp() {
        frontSpark.set(Constants.NeckConstants.NECK_FRONT_SPEED);
        backSpark.set(Constants.NeckConstants.NECK_BACK_SPEED);
    }

    public void moveUp(double speed) {
        frontSpark.set(speed);
        backSpark.set(speed);
    }

    public void moveFrontUp() {
        frontSpark.set(Constants.NeckConstants.NECK_FRONT_SPEED);
    }

    public void moveBackUp() {
        backSpark.set(Constants.NeckConstants.NECK_BACK_SPEED);
    }

    public void moveDown() {
        frontSpark.set(-Constants.NeckConstants.NECK_FRONT_OUT_SPEED);
        backSpark.set(-Constants.NeckConstants.NECK_BACK_OUT_SPEED);
    }

    public void stopNeck() {
        frontSpark.stopMotor();
        backSpark.stopMotor();
    }

    public void stopFrontMotor() {
        frontSpark.stopMotor();
    }

    public void stopBackMotor() {
        backSpark.stopMotor();
    }

    public boolean getLowBeamBreak() {
        return lowBeamBreak.get();
    }

    public boolean getHighBeamBreak() {
        return highBeamBreak.get();
    }
}

