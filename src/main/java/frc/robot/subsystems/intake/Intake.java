package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private IntakeIOInputsAutoLogged intput = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setPivotAngle(double angle) {
        io.setPivotPosition(angle);
        Logger.recordOutput("OTBIntake/Pivot", angle);
    }

    public void setIntakeSpeed(double speed) {
        io.setIntakeSpeed(speed);
        Logger.recordOutput("OTBIntake/Intake", speed);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("OTBIntake", inputs);
    }    
}
