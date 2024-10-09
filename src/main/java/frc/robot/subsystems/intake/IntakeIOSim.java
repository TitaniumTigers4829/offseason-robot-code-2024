package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim {
    DCMotorSim intakeSim = new DCMotorSim(null, 0, 0);
    SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getFalcon500(2), 0, 0, 0, 0, 0, false, 0);
}
