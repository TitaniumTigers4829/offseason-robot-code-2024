package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.extras.VirtualSubsystem;

public class Elevator extends VirtualSubsystem {
    
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
    }

    public void setElevatorPosition(double position) {
        Logger.recordOutput("Elevator", position);
        io.setElevatorPosition(position);
    }

    @Override
    public void periodic(double dt, boolean enabled) {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }
}
