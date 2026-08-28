package org.firstinspires.ftc.teamcode.LogicNodes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Trajectories.FarRed.shootPos;
import static org.firstinspires.ftc.teamcode.Trajectories.FarRed.spike3Pos;
import static org.firstinspires.ftc.teamcode.Trajectories.FarRed.tunnelPose;
import static org.firstinspires.ftc.teamcode.Trajectories.FarRed.loadingPose;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

public class FarRed {
    public ElapsedTime timer,globalTimer;
    public Chassis chassis;
    public Intake intake;
    public Shooter shooter;
    public Odo odo;
    Node shoot,park,loadingZone,tunnel,spike3;
    public Node currentNode,prevNode;
    public FarRed(HardwareMap hardwareMap){
        Hardware.init(hardwareMap);
        timer = new ElapsedTime(); timer.startTime();
        globalTimer = new ElapsedTime(); globalTimer.startTime();
        chassis = new Chassis(Chassis.State.PID);
        intake = new Intake();
        shooter = new Shooter(Shooter.State.ACTIVE);
        odo = new Odo();
        intake.storage.setState(Storage.State.TRANSFER);
        shoot = new Node("shoot");
        park = new Node("park");
        loadingZone = new Node("loadingZone");
        tunnel = new Node("tunnel");
        spike3 = new Node("spike3");
        shooter.turret.setState(Turret.State.RED);
        currentNode = shoot;

        shoot.addConditions(
                ()->{
                    Chassis.kp = 0.0065;
                    if (prevNode == loadingZone) chassis.setTargetPosition(shootPos[0]);
                    else chassis.setTargetPosition(shootPos[0]);
                    if((intake.storage.getState()== Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER) && !intake.storage.isMoving() && !chassis.inPosition(100,100,0.3))intake.setState(Intake.State.REVERSE);
                    if ((intake.storage.getState()!= Storage.State.TRANSFER && intake.storage.getState()!= Storage.State.GOINGTRANSFER) && !intake.storage.isMoving()) intake.storage.setState(Storage.State.GOINGTRANSFER);
                    if (chassis.inPosition(100, 100, 0.1) && intake.storage.getState() == Storage.State.TRANSFER && intake.latch.getState() == Latch.State.TRANSFER && shooter.flyWheel.isReady() && !intake.latch.isMoving()) intake.setState(Intake.State.SHOOT);
                },
                ()->{
                    return intake.storage.getState() == Storage.State.GOINGBALL1;
                },
                new Node[]{loadingZone,spike3,loadingZone,loadingZone,tunnel,loadingZone,loadingZone,tunnel,loadingZone,loadingZone,loadingZone,park}
        );

        spike3.addConditions(
                ()->{
                    if (spike3.index == 1) Chassis.kp = 0.000935;
                    chassis.setTargetPosition(spike3Pos[Math.min(spike3.index,spike3Pos.length-1)]);
                    intake.setState(Intake.State.INTAKE);
                },
                ()->{
                    if (spike3.index == 0 && chassis.inPosition(120,120,0.1)) return true;
                    return chassis.inPosition(60,60,0.1) || intake.storage.getState() == Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER;
                },
                new Node[]{spike3,shoot}
        );

        tunnel.addConditions(
                ()->{
                    chassis.setTargetPosition(tunnelPose[tunnel.index % 2]);
                    if (!chassis.inPosition(60,60,0.1)) timer.reset();
                    intake.setState(Intake.State.INTAKE);

                },
                ()->{
                    return timer.seconds()>0.6 || intake.storage.getState() == Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER;
                },
                new Node[]{shoot}
        );
        loadingZone.addConditions(
                ()->{
                    chassis.setTargetPosition(loadingPose[Math.min(loadingZone.index,loadingPose.length-1)]);
                    intake.setState(Intake.State.INTAKE);
                    if (!chassis.inPosition(60,60,0.1)) timer.reset();
                    if (intake.storage.state == Storage.State.GOINGTRANSFER || intake.storage.state == Storage.State.TRANSFER) loadingZone.index = 2;
                },
                ()->{
                    return timer.seconds()>0.65 || intake.storage.getState() == Storage.State.GOINGTRANSFER || intake.storage.getState() == Storage.State.TRANSFER;
                },
                new Node[]{loadingZone,loadingZone,loadingZone,shoot}
        );
        park.addConditions(
                ()->{
                    Chassis.stop = false;
                    chassis.setTargetPosition(spike3Pos[0]);
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
        chassis.update();
        odo.update();
        intake.update();
        shooter.update();
        if (globalTimer.seconds()>29.8 && currentNode!=shoot)currentNode = park;
        if (currentNode.transition()){
            prevNode = currentNode;
            currentNode = currentNode.next[Math.min(currentNode.index++,currentNode.next.length-1)];
        }
    }
}
