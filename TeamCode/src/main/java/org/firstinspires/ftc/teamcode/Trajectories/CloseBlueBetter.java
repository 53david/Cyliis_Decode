package org.firstinspires.ftc.teamcode.Trajectories;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.firstinspires.ftc.teamcode.Wrappers.LimeLight;

public class CloseBlueBetter {
    Odo odo;
    ElapsedTime timer;
    Chassis chassis;
    Shooter shooter;
    Intake intake;
    LimeLight limeLight;
    public static Pose2D[] shootPos = {
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),

    };
    public static Pose2D[] gatePos = {
            new Pose2D(),
            new Pose2D(),
            new Pose2D(),
    };
    public static Pose2D spike1Pos = new Pose2D(0,0,0);
    public static Pose2D spike2Pos =new Pose2D();
    public static Pose2D loadingPos= new Pose2D();
    Node shoot,spike1,spike2,gate,loading;
    public Node currentNode;
    public CloseBlueBetter(HardwareMap hardwareMap){
        odo = new Odo();
        chassis = new Chassis(Chassis.State.PID);
        intake = new Intake();
        shooter = new Shooter();
        limeLight = new LimeLight();
        Turret.allienceState = Turret.AllianceState.BLUE;
        Storage.state=Storage.State.TRANSFER;
        odo.reset();
        shoot.addConditions(
                ()->{
                    if ((Intake.state == Intake.State.INTAKE || Intake.state == Intake.State.REVERSE) && chassis.inPosition(100,100,0.2)){
                        Intake.state = Intake.State.IDLE;
                    }
                    else if (Storage.state == Storage.State.TRANSFER && !chassis.inPosition(100,100,0.2)){
                        Intake.state = Intake.State.REVERSE;
                    }
                    Shooter.state = Shooter.State.SHOOT;
                    chassis.setTargetPosition(shootPos[Math.min(shoot.index,shootPos.length-1)]);

                },
                ()->{
                    if (Storage.state == Storage.State.RESET){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{spike1,spike2,gate,gate,gate,loading}
        );
        spike1.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(spike1Pos);

                },
                ()->{
                    if (chassis.inPosition(30,30,0.1) || Storage.state == Storage.State.TRANSFER){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        spike2.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(spike2Pos);

                },
                ()->{
                    if (chassis.inPosition(30,30,0.1) || Storage.state == Storage.State.TRANSFER){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        loading.addConditions(
                ()->{
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(loadingPos);
                    Shooter.state = Shooter.State.IDLE;
                },
                ()->{
                    if (chassis.inPosition(30,30,0.1) || Storage.state == Storage.State.TRANSFER) {
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        gate.addConditions(
                ()->{
                   chassis.setTargetPosition(gatePos[Math.min(gate.index, gatePos.length-1)]);
                   Intake.state = Intake.State.INTAKE;
                   Shooter.state = Shooter.State.IDLE;

                },
                ()->{
                    if (gate.index!=2 && chassis.inPosition(100,100,0.2)){
                        timer.reset();
                        return true;
                    }
                    else if (gate.index == 2 && (Storage.state == Storage.State.TRANSFER || timer.seconds()>1.5)){
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
        chassis.update();
        limeLight.update();
        shooter.update();
        intake.update();
        if (currentNode.transition()){
            currentNode = currentNode.next[Math.min(currentNode.index++,currentNode.next.length-1)];
        }
    }
}
