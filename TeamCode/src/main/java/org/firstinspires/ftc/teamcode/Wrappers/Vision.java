package org.firstinspires.ftc.teamcode.Wrappers;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.webcam;

import android.util.Size;

import com.bylazar.camerastream.PanelsCameraStream;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class Vision {
    public enum State{
        BLUE,
        RED,
    };
    public static int gain = 10,exposure = 1000,temp = 60000;
    public static double allianceID;
    public static State state;
    public VisionPortal visionPortal;
    public AprilTagProcessor tagProcessor;
    public double fx,fy,cx,cy;
    public Vision(){
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setDrawCubeProjection(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(fx,fy,cx,cy)
                .build();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setLiveViewContainerId(0)
                .build();

        PanelsCameraStream.INSTANCE.startStream(visionPortal, 10);
    }
    public void update(){
        stateUpdate();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING){
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            exposureControl.setMode(ExposureControl.Mode.Manual);
            whiteBalanceControl.setMode(WhiteBalanceControl.Mode.MANUAL);
            exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS);
            whiteBalanceControl.setWhiteBalanceTemperature(temp);
            gainControl.setGain(gain);
        }
        stateUpdate();
    }
    public void stateUpdate(){
        switch (state){
            case BLUE:
                allianceID = 20;
                break;
            case RED:
                allianceID = 24;
                break;

        }
    }
    public double getTagAngle(){

        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == allianceID) {
                return -tag.ftcPose.bearing;
            }
        }
        return 1e9;
    }
    public double getDistance(){

        for(AprilTagDetection tag:tagProcessor.getDetections()){
            if (tag.id == allianceID) {
                return tag.ftcPose.range;
            }
        }
        return 1e9;
    }
    public double tagID(){

        for(AprilTagDetection tag:tagProcessor.getDetections()){
            return tag.id;
        }
        return 1e9;
    }
}
