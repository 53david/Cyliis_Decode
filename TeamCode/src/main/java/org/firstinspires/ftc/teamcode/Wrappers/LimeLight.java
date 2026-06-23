package org.firstinspires.ftc.teamcode.Wrappers;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.limelight3A;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;

@Configurable
public class LimeLight {
    public enum State{
        RED,
        BLUE,
    };
    public enum StreamState{
        CLOSE,
        STREAM,
    }
    public boolean ok;
    public static StreamState streamState;
    public static State state;
    public static double allianceID= 0;
    double cameraAngle = 0;
    double cameraHeight = 0;
    public static double distance = 0;
    public static double ty = 0;
    public static double tx = 0;
    public static double ta = 0;
    int prevIndex = 0;
    public static int index = 0;
    public static LLResult result;

    public LimeLight(StreamState state1, State state2){
        streamState = state1;
        state = state2;
    }
    public void update(){
        stateUpdate();
        if (streamState != StreamState.CLOSE) {
            result = limelight3A.getLatestResult();
            if (prevIndex != index && limelight3A.isRunning()) {
                limelight3A.pipelineSwitch(index);
            }
            prevIndex = index;
            if (result!=null && result.isValid()) {
                ty = result.getTy();
                tx = result.getTx();
                ta = result.getTa();
                distance = cameraHeight / Math.tan(Math.toRadians(cameraAngle + ty));

            }
        }
    }
    public void stateUpdate(){
        switch (state){
            case RED:
                allianceID = 24;
                break;
            case BLUE:
                allianceID = 20;
                break;

        }
        switch (streamState){
            case CLOSE:
                limelight3A.close();
                break;
            case STREAM:
                limelight3A.start();
                break;
        }
    }
    public static double getDistance(){
        if (result!=null && result.isValid()) {
            return distance;
        }
        return 1e9;
    }
    public static double getHeading(){
        if (result!=null && result.isValid()){
            return tx;
        }
        return 1e9;
    }
    public static double tagID(){
        if (result!=null && !result.getFiducialResults().isEmpty()) {
            return result.getFiducialResults().get(0).getFiducialId();
        }
        return 1e9;
    }
    public static double getArea(){
        if (result!=null && result.isValid()){
            return ta;
        }
        return 1e9;
    }
    public static boolean isActive(){
        return limelight3A.isRunning();
    }
    public static double getTagAngle(){
        if (result!=null && !result.getFiducialResults().isEmpty()) {
            for (var tag : result.getFiducialResults()) {
                if (tag.getFiducialId() == allianceID) {
                    return tag.getTargetXDegrees();
                }
            }
        }
        return 1e9;
    }
    public static double getPipeline(){
        return index;
    }
    public void setIndex(int x){
        index = x;
    }

}
