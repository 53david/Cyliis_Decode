package org.firstinspires.ftc.teamcode.LogicNodes;

import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.gatePos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.parkPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.shootPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.spike1Pos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.spike2Pos;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class CloseBlue {
    Odo odo;
    ElapsedTime timer;
    Chassis chassis;
    Shooter shooter;
    Intake intake;
    boolean ok = true;
    boolean ok1 = true;
    Node shoot,spike1,spike2,gate,loading,park;
    public Node currentNode;
    public CloseBlue(HardwareMap hardwareMap){
        timer = new ElapsedTime();
        Initializer.start(hardwareMap);
        odo = new Odo();
        chassis = new Chassis(Chassis.State.PID);
        intake = new Intake();
        shooter = new Shooter(Shooter.State.SHOOT);
        Turret.allianceState = Turret.AllianceState.BLUE;
        timer.startTime();
        shoot = new Node("shoot");
        spike1 =  new Node("spike1");
        spike2 = new Node("spike2");
        gate = new Node("gate");
        loading = new Node("loading");
        park = new Node("park");
        currentNode = shoot;
        Storage.state = Storage.State.TRANSFER;
        shoot.addConditions(
                ()->{
                    gate.reset();
                    chassis.setTargetPosition(shootPos[Math.min(shoot.index,shootPos.length-1)]);
                    if (Storage.state != Storage.State.TRANSFER){
                        Intake.state = Intake.State.INTAKE;
                    }
                    if (Storage.state == Storage.State.TRANSFER && !Storage.IsStorageSpinning()){
                        Intake.state = Intake.State.REVERSE;
                    }
                    if (!Storage.IsStorageSpinning() && Storage.state!= Storage.State.TRANSFER && Storage.state!= Storage.State.SHOOT
                            && Storage.state != Storage.State.RESET && chassis.inPosition(60,60,0.4)){
                        Storage.state = Storage.State.TRANSFER;
                    }

                    if (chassis.inPosition(60,60,0.05) && FlyWheel.isReady() && Storage.state == Storage.State.TRANSFER && Storage.state != Storage.State.RESET){
                        Intake.state = Intake.State.IDLE;
                        Storage.state = Storage.State.SHOOT;
                    }
                },
                ()->{
                   return Storage.state == Storage.State.RESET;
                },
                new Node[]{spike2,gate,gate,spike1,gate,gate,park}
        );
        spike1.addConditions(
                ()->{
                    chassis.setTargetPosition(spike1Pos[Math.min(spike1.index,spike1Pos.length-1)]);
                    Intake.state = Intake.State.INTAKE;
                },
                ()->{
                    return chassis.inPosition(30,30,0.2) || Storage.state == Storage.State.TRANSFER;
                },
                new Node[]{spike1,shoot}
        );
        spike2.addConditions(
                ()->{
                    chassis.setTargetPosition(spike2Pos[Math.min(spike2.index,spike1Pos.length-1)]);
                    Intake.state = Intake.State.INTAKE;
                },
                ()->{
                    return chassis.inPosition(30,30,0.2) || Storage.state == Storage.State.TRANSFER;
                },
                new Node[]{spike2,shoot}
        );
        gate.addConditions(
                ()->{
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(gatePos[Math.min(gate.index, gatePos.length-1)]);
                    if (gate.index == 1 && chassis.inPosition(100,100,0.1)){
                        Chassis.state = Chassis.State.IDLE;
                    }
                },
                ()->{
                    if (gate.index ==0) {
                        timer.reset();
                        return chassis.inPosition(200, 200, 0.6);
                    }
                    else if (gate.index == 1){
                        Chassis.state = Chassis.State.PID;
                       return timer.seconds()>2.5 || Storage.state == Storage.State.TRANSFER;
                    }
                    return false;
                },
                new Node[]{gate,shoot}
        );
        park.addConditions(
                ()->{
                    Intake.state = Intake.State.IDLE;
                    chassis.setTargetPosition(parkPos);
                },
                ()->{
                    return true;
                },
                new Node[]{park}
        );
    }
    public void update(){
        currentNode.run();
        chassis.update();
        shooter.update();
        odo.update();
        intake.update();
        if (currentNode.transition()){
            currentNode = currentNode.next[Math.min(currentNode.index++,currentNode.next.length-1)];
        }
        telemetryM.addData("Current Node",currentNode.getName());
        telemetryM.update();
    }
}
