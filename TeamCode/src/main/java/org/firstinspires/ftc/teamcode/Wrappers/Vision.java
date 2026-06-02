package org.firstinspires.ftc.teamcode.Wrappers;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.limelight3A;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;

@Configurable
public class Vision {
    double cameraAngle = 0;
    double cameraHeight = 0;
    public static double area = 0;
    public static double ty = 0;
    public static double tx = 0;
    public static double ta = 0;
    int prevIndex = 0;
    public static int index = 0;
    public static LLResult result;
    public Vision(){
        limelight3A.start();
    }
    public void update(){
        result = limelight3A.getLatestResult();
        if (prevIndex!=index && limelight3A.isRunning()){
            limelight3A.pipelineSwitch(index);
        }
        prevIndex = index;
        if (result.isValid() && result!=null){
            ty = result.getTy();
            tx = result.getTx();
            ta = result.getTa();
            area = cameraHeight / Math.toRadians(cameraAngle-ty);

        }
    }
    public static double getDistance(){
        if (result!=null && result.isValid()) {
            return area;
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

}
