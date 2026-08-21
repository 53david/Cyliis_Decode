package org.firstinspires.ftc.teamcode.LogicNodes;

import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.afterCollectPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.gatePos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.goingGatePos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.shootPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.spike1Pos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.spike2Pos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

public class CloseBlue {
    public Odo odo;
    public ElapsedTime gateTimer,timer,globalTimer;
    public Chassis chassis;
    public Shooter shooter;
    public Intake intake;

    public Node shoot,spike1,spike2,goingGate,gate,afterCollect,park;
    public Node currentNode;
    public CloseBlue(HardwareMap hardwareMap){
        gateTimer = new ElapsedTime();timer = new ElapsedTime(); globalTimer =new ElapsedTime();
        gateTimer.startTime();timer.startTime();globalTimer.startTime();
        Hardware.init(hardwareMap);
        odo = new Odo();
        chassis = new Chassis(Chassis.State.PID);
        intake = new Intake();
        shooter = new Shooter(Shooter.State.ACTIVE);
        shoot = new Node("shoot");
        spike1 =  new Node("spike1");
        spike2 = new Node("spike2");
        gate = new Node("gate");
        goingGate = new Node("goingGate");
        afterCollect = new Node("affterCollect");
        park = new Node("park");
        currentNode = shoot;
        intake.storage.setState(Storage.State.TRANSFER);
        shooter.turret.setState(Turret.State.BLUE);
        shoot.addConditions(
                ()->{
                    chassis.setTargetPosition(shootPos[Math.min(shoot.index,shootPos.length-1)]);
                    if((intake.storage.getState()== Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER) && !intake.storage.isMoving() && !chassis.inPosition(220,220,0.3))intake.setState(Intake.State.REVERSE);
                    if ((intake.storage.getState()!= Storage.State.TRANSFER && intake.storage.getState()!= Storage.State.GOINGTRANSFER) && !intake.storage.isMoving()) intake.storage.setState(Storage.State.GOINGTRANSFER);
                    if (chassis.inPosition(180,180,0.1) && intake.storage.getState() == Storage.State.TRANSFER && intake.latch.getState() == Latch.State.TRANSFER && shooter.flyWheel.isReady() && !intake.latch.isMoving())intake.setState(Intake.State.SHOOT);

                },
                ()->{
                  return intake.storage.getState() == Storage.State.GOINGBALL1;
                },
                new Node[]{spike2,goingGate,goingGate,spike1,goingGate,goingGate,goingGate,goingGate,park}
        );
        goingGate.addConditions(
                ()->{
                    chassis.setTargetPosition(goingGatePos);
                    intake.setState(Intake.State.IDLE);
                },
                ()->{
                    return chassis.inPosition(200,200,0.1);
                },
                new Node[]{gate}
        );
        gate.addConditions(
                ()->{
                    chassis.setTargetPosition(gatePos);
                    intake.setState(Intake.State.INTAKE);
                    if (!chassis.inPosition(90,90,0.3)){timer.reset(); gateTimer.reset();}
                    if (chassis.inPosition(90,90,0.3) && timer.seconds()>0.085)Chassis.stop = true;
                },
                ()->{
                    if (intake.storage.getState() == Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER || gateTimer.seconds()>2.5){

                        Chassis.stop= false;
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        afterCollect.addConditions(
                ()->{
                    chassis.setTargetPosition(afterCollectPos);
                    if((intake.storage.getState()== Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER) && !intake.storage.isMoving())intake.setState(Intake.State.REVERSE);
                    else if (!chassis.inPosition(30,30,0.1)) intake.setState(Intake.State.IDLE);
                },
                ()->{
                    return chassis.inPosition(30,30,0.1);
                },
                new Node[]{shoot}
        );
        spike1.addConditions(
                ()->{
                    chassis.setTargetPosition(spike1Pos[Math.min(spike1.index,spike1Pos.length-1)]);
                    intake.setState(Intake.State.INTAKE);
                },
                ()->{
                    return chassis.inPosition(30,30,0.1) || intake.storage.getState() == Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER;
                },
                new Node[]{spike1,shoot}
        );
        spike2.addConditions(
                ()->{
                    shooter.turret.pause = false;
                    chassis.setTargetPosition(spike2Pos[Math.min(spike2.index,spike2Pos.length-1)]);
                    intake.setState(Intake.State.INTAKE);
                },
                ()->{
                    return chassis.inPosition(30,30,0.1) || intake.storage.getState() == Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER;
                },
                new Node[]{spike2,shoot}
        );
        park.addConditions(
                ()->{
                    Chassis.stop = false;
                    chassis.setTargetPosition(spike2Pos[0]);
                    intake.setState(Intake.State.IDLE);
                    shooter.setState(Shooter.State.IDLE);
                },
                ()->{
                    return chassis.inPosition(30,30,0.1);
                },
                new Node[]{park}
        );



    }
    public void update(){
        currentNode.run();
        if (shoot.index == 0) shooter.turret.pause = true;
        else  shooter.turret.pause = false;
        chassis.update();
        shooter.update();
        odo.update();
        intake.update();
        if (globalTimer.seconds()>29.5 && intake.getState()!= Intake.State.SHOOT)currentNode = park;
        if (currentNode.transition()){
            currentNode = currentNode.next[Math.min(currentNode.index++,currentNode.next.length-1)];
        }
    }
}
