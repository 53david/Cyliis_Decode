package org.firstinspires.ftc.teamcode.Wrappers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
public class Limelight {
    ElapsedTime timer;
    public static double x = 0.25;
    public enum HiveState{
        UP,
        DOWN,
    }
    public enum State{
        ACTIVE(),
        STOP;
        State nextState;
        State(){
            this.nextState = this;
        }
        State(State nextState){
            this.nextState = nextState;
        }
    }

    public double a;
    public int index,pipelineIndex,prevIndex,pollingRate = 100,prevPollingRate;
    public State state;
    public HiveState hiveState = HiveState.DOWN;
    Limelight3A limelight;
    public Limelight(State state){
        timer = new ElapsedTime();
        timer.startTime();
        this.state = state;
        limelight = Hardware.limelight;

    }
    public void updateState(){
        switch (state){
            case ACTIVE:
                if (!limelight.isRunning()) limelight.start();
                break;
            case STOP:
                if (limelight.isRunning()) limelight.stop();
                break;
        }
        switch (hiveState){
            case UP:
            case DOWN:
                break;
        }
        if (Odo.predictedX < -1600) hiveState = HiveState.UP;
        else hiveState = HiveState.DOWN;
        if (limelight.isRunning() && state == State.ACTIVE) state = state.nextState;
    }
    public void update(){
        updateState();
        updateHardware();
        prevIndex = pipelineIndex;
        prevPollingRate = pollingRate;
    }
    public void updateHardware(){
        LLResult result = limelight.getLatestResult();
        if (result !=null && result.isValid()){
            List<LLResultTypes.FiducialResult> list = result.getFiducialResults();
            if (!list.isEmpty()){
                timer.reset();
                if (list.get(0).getFiducialId() == 42) {index = 0; timer.reset();}
                if (list.get(0).getFiducialId() == 41) {index = 1; timer.reset();}
                if (list.get(0).getFiducialId() == 37) {index = 2; timer.reset();}
                if (list.get(0).getFiducialId() == 30) {index = 3; timer.reset();}

            }
        }
        if (Odo.avgVel()>100) timer.reset();
        if (timer.seconds()>2.5){
            if (hiveState == HiveState.DOWN && limelight.getStatus().getPipelineIndex() == 5){index = 1; timer.reset();}
            if (hiveState == HiveState.UP && limelight.getStatus().getPipelineIndex() == 5){index = 0; timer.reset();}
            if (hiveState == HiveState.DOWN && limelight.getStatus().getPipelineIndex() == 4){index = 3; timer.reset();}
            if (hiveState == HiveState.UP && limelight.getStatus().getPipelineIndex() == 4){index = 2; timer.reset();}

        }
    }
    public State getState(){
        return state;
    }
    public void setState(State state){
        this.state = state;
    }
    public int getPositionIndex(){
        return index;
    }
    public void setPipeLineIndex(int pipelineIndex){
        if (prevIndex != pipelineIndex) limelight.pipelineSwitch(pipelineIndex);
    }
    public int getPipeLineIndex(){
        return pipelineIndex;
    }
    public HiveState getHiveState(){
        return hiveState;
    }
    public double getTime(){
        return timer.seconds();
    }
}
