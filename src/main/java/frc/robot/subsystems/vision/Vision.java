package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    private VisionIO io[];
    private VisionIOInputsAutoLogged inputs[];
    private boolean[] shouldUpdate = new boolean[] {true, true, true};

    private final Translation3d redSpeaker = new Translation3d(16.379342, 5.547868, 2.06);
    private final Translation3d blueSpeaker = new Translation3d(0.1619, 5.547868, 2.06);
    
    @AutoLogOutput
    private Translation3d speakerPosition = new Translation3d();

    public static Vision instance;
    
    public static Vision getInstance() {
        return instance;
    }

    public static Vision initialize(VisionIO... io) {
        if (instance == null) {
            instance = new Vision(io);
        }
        return instance;
    }

    private Vision(VisionIO visionIO[]) {
        io = visionIO;

        inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        setSpeakerPos();
    }

    public double getDistancetoSpeaker(Pose2d robotPose) {
        if(speakerPosition == null){
            return 0;
        }
        return robotPose.getTranslation().getDistance(speakerPosition.toTranslation2d());
    }

    public void setSpeakerPos(){
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            if (ally.get() == Alliance.Red) {
                speakerPosition = redSpeaker;
            } else if (ally.get() == Alliance.Blue) {
                speakerPosition = blueSpeaker;
            }
        } else {
            speakerPosition = new Translation3d();
        }
    }

    public Translation3d getSpeakerPos() {
        return speakerPosition;
    }

    public void setReferencePose(Pose2d reference) {
        for(int i = 0; i < io.length; i++) {
            io[i].setReferencePose(reference);
        }
    }

    public boolean canSeeAprilTags() {
        return false;
    }

    public void disableUpdates(int id){
        shouldUpdate[id] = false;
    }

    public void enableUpdates(int id){
        shouldUpdate[id] = true;
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + VisionConstants.cameraIds[i], inputs[i]);
        }
        
        setSpeakerPos();
    }
}