package org.firstinspires.ftc.teamcode.LogicNodes;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseRed.gatePos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseRed.parkPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseRed.shootPos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseRed.spike1Pos;
import static org.firstinspires.ftc.teamcode.Trajectories.CloseRed.spike2Pos;

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
public class CloseRed{
    Node shoot,spike1,spike2,gate,loading,park;
    public Node currentNode;
    Intake intake;
    ElapsedTime timer;
    Odo odo;
    Chassis chassis;
    Shooter shooter;
    public boolean ok = true;
    public CloseRed(HardwareMap hardwareMap){
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
    odo.reset();
    shoot.addConditions(
            ()->{
                chassis.setTargetPosition(shootPos[Math.min(shoot.index,shootPos.length-1)]);
                if ((Intake.state == Intake.State.INTAKE || Intake.state == Intake.State.REVERSE) && Storage.state == Storage.State.TRANSFER){
                    Intake.state = Intake.State.IDLE;
                }
                if (chassis.inPosition(60,60,0.5) && FlyWheel.isReady() && ok){
                    Storage.state = Storage.State.SHOOT;
                    ok = false;
                }
            },
            ()->{
                if (Storage.state == Storage.State.RESET && !ok) {
                    ok = true; timer.reset();
                    gate.index =0;
                    return true;
                }
                return false;
            },
            new Node[]{spike2,gate,gate,spike1,gate,gate,gate,park}
    );
    spike1.addConditions(
            ()->{
                chassis.setTargetPosition(spike1Pos);
                Intake.state = Intake.State.INTAKE;

            },
            ()->{
                if (chassis.inPosition(30,30,0.1)){
                    Intake.state = Intake.State.REVERSE;
                    Storage.state = Storage.State.TRANSFER;
                    return true;
                }
                return false;
            },
            new Node[]{shoot}
    );
    spike2.addConditions(
            ()->{
                chassis.setTargetPosition(spike2Pos);
                Intake.state = Intake.State.INTAKE;
            },
            ()->{
                if (chassis.inPosition(30,30,0.1)){
                    Storage.state = Storage.State.TRANSFER;
                    return true;
                }
                return false;
            },
            new Node[]{shoot}
    );
    gate.addConditions(
            ()->{
                Intake.state = Intake.State.INTAKE;
                chassis.setTargetPosition(gatePos[Math.min(gate.index, gatePos.length-1)]);
            },
            ()->{
                if (gate.index ==0) {
                    timer.reset();
                    return chassis.inPosition(30, 30, 0.08);
                }
                else if (gate.index == 1){
                    if (Storage.state == Storage.State.TRANSFER){
                        Intake.state = Intake.State.REVERSE;
                        return true;
                    }
                    else if (timer.seconds()>3.25){
                        Intake.state = Intake.State.REVERSE;
                        Storage.state = Storage.State.TRANSFER;
                        return true;
                    }
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
