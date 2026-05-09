package org.firstinspires.ftc.teamcode.Trajectories;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class CloseRed {
    public ElapsedTime timer;
    public Pose2D shootPos = new Pose2D(0,0,0);
    public Pose2D[] gatePos = {
            new Pose2D(0, 0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public Pose2D[] spike1Pos = {
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public Pose2D[] spike2Pos= {
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    Shooter shooter;
    Storage storage;
    Chassis chassis;
    Odo odo;
    Intake intake;
    Node shoot,gate,spike1,spike2;
    public Node currentNode;
    public CloseRed(HardwareMap hardwareMap){
        Initializer.start(hardwareMap);
        timer = new ElapsedTime();
        timer.startTime();
        Shooter.state = Shooter.State.SHOOT;
        Turret.allienceState = Turret.AllianceState.RED;
        Storage.state = Storage.State.TRANSFER;
        storage = new Storage();
        shooter = new Shooter();
        chassis = new Chassis(Chassis.State.PID);
        odo = new Odo();
        intake = new Intake();
        shoot = new Node("shoot");
        gate = new Node("gate");
        spike1 =new Node("spike1");
        spike2 = new Node("spike2");
        currentNode = shoot;
        shoot.addConditions(
                ()-> {
                    if ((Intake.state == Intake.State.INTAKE || Intake.state == Intake.State.REVERSE)
                            && !storage.IsStorageSpinning()){
                        Intake.state = Intake.State.IDLE;
                    }
                    Shooter.state = Shooter.State.SHOOT;
                    chassis.setTargetPosition(shootPos);
                    if (chassis.inPosition(40,40,0.25) && Storage.state == Storage.State.TRANSFER){
                        Storage.state = Storage.State.SHOOT;
                    }
                },
                ()->{
                    if (Storage.state == Storage.State.RESET){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{spike2,gate,gate,gate,spike1}
        );
        gate.addConditions(
                ()->{
                    if (Storage.state != Storage.State.TRANSFER) {
                        Intake.state = Intake.State.INTAKE;
                    }
                    else {
                        Intake.state = Intake.State.REVERSE;
                    }
                    chassis.setTargetPosition(gatePos[Math.min(gate.index, gatePos.length-1)]);
                    Shooter.state = Shooter.State.IDLE;

                },
            ()->{
            if (Storage.state == Storage.State.TRANSFER){
                timer.reset();
                return true;
            }
            return false;
        },
            new Node[]{shoot}
        );
        spike1.addConditions(
                ()->{
                    if (Storage.state != Storage.State.TRANSFER) {
                        Intake.state = Intake.State.INTAKE;
                    }
                    else {
                        Intake.state = Intake.State.REVERSE;
                    }
                    chassis.setTargetPosition(spike1Pos[Math.min(spike1.index,spike1Pos.length-1)]);
                    Shooter.state = Shooter.State.IDLE;

                },
                ()->{
                    if (Storage.state == Storage.State.TRANSFER){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        spike2.addConditions(
                ()->{
                    if (Storage.state != Storage.State.TRANSFER) {
                        Intake.state = Intake.State.INTAKE;
                    }
                    else {
                        Intake.state = Intake.State.REVERSE;
                    }
                    chassis.setTargetPosition(spike2Pos[Math.min(spike2.index,spike2Pos.length-1)]);
                    Shooter.state = Shooter.State.IDLE;

                },
                ()->{
                    if (Storage.state == Storage.State.TRANSFER){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
    }
    public void update(){
        currentNode.run();
        odo.update();
        shooter.update();
        chassis.update();
        intake.update();
        if (currentNode.transition()){
            currentNode = currentNode.next[Math.min(currentNode.index++,currentNode.next.length-1)];
        }
    }

}
