package org.firstinspires.ftc.teamcode.LogicNodes;

import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.gatePos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.parkPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.shootPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.spike1Pos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseBlue.spike2Pos;

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
        Storage.state = Storage.State.TRANSFER;
        timer.startTime();
        shoot = new Node("shoot");
        spike1 =  new Node("spike1");
        spike2 = new Node("spike2");
        gate = new Node("gate");
        loading = new Node("loading");
        park = new Node("park");
        currentNode = shoot;
        shoot.addConditions(
                ()->{
                    chassis.setTargetPosition(shootPos[Math.min(shoot.index,shootPos.length-1)]);
                    if (!Storage.IsStorageSpinning() && !chassis.inPosition(70,70,0.4)){
                        Storage.state = Storage.State.TRANSFER;
                        Intake.state = Intake.State.REVERSE;
                    }
                    if (Latch.state != Latch.State.TRANSFER && ok && !Storage.IsStorageSpinning()){
                        Latch.state = Latch.State.TRANSFER;
                    }

                    if (chassis.inPosition(30,30,0.05) && Odo.xVelocity<20 && Odo.yVelocity<20 && FlyWheel.isReady() && ok){
                        Intake.state = Intake.State.INTAKE;
                        Storage.state = Storage.State.SHOOT;
                        ok = false;
                    }
                },
                ()->{
                    if (Storage.state == Storage.State.RESET) {
                        ok = true; timer.reset();
                        Latch.state = Latch.State.IDLE;
                        gate.index =0;
                        return true;
                    }
                    return false;
                },
                new Node[]{spike2,gate,gate,spike1,gate,gate,gate,park}
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
                },
                ()->{
                    if (gate.index ==0) {
                        timer.reset();
                        return chassis.inPosition(200, 200, 0.6);
                    }
                    else if (gate.index == 1){
                       return timer.seconds()>2.5 || Storage.state == Storage.State.TRANSFER;
                    }
                    return false;
                },
                new Node[]{gate,shoot}
        );
        park.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
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

    }
}
