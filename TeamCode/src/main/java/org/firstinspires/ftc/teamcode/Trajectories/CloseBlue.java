package org.firstinspires.ftc.teamcode.Trajectories;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class CloseBlue {
    ElapsedTime timer = new ElapsedTime();
    Storage storage;
    public Chassis chassis;
    public Intake intake;
    public Shooter shooter;
    public Odo odo;
    public static Pose2D shootPos = new Pose2D(-1450, -460, Math.PI/2);
    public static Pose2D loadingPos = new Pose2D(0,0,0);
    public static Pose2D[] gatePos = {
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public static Pose2D[] spike1Pos={
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public static Pose2D[] spike2Pos={
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public static Pose2D parkPos = new Pose2D(0,0,0);
    Node shoot,spike1,loading,spike2,gate,park;
    public Node currentNode;

    public CloseBlue(HardwareMap hardwareMap){
        Initializer.start(hardwareMap);
         storage = new Storage();
         chassis = new Chassis(Chassis.State.PID);
         intake = new Intake();
         shooter = new Shooter();
         odo = new Odo();
        Storage.state = Storage.State.TRANSFER;
        Turret.allienceState = Turret.AllianceState.BLUE;
        shoot = new Node("shoot");
        spike1 = new Node("spike1");
        spike2 = new Node("spike2");
        gate = new Node("gate");
        currentNode = shoot;
        shoot.addConditions(
                ()->{
                    if ((Intake.state == Intake.State.INTAKE || Intake.state == Intake.State.REVERSE)&& !storage.IsStorageSpinning()){
                        Intake.state = Intake.State.IDLE;
                    }
                    chassis.setTargetPosition(shootPos);
                    Shooter.state = Shooter.State.SHOOT;
                    if (chassis.inPosition(40,40,0.13) && Math.abs(Initializer.pp.getVelX(DistanceUnit.MM))<=25
                            && Math.abs(Initializer.pp.getVelY(DistanceUnit.MM))<=25){
                                if (Storage.state == Storage.State.TRANSFER) {
                                    Storage.state = Storage.State.SHOOT;
                                }
                                else {
                                    Storage.state = Storage.State.TRANSFER;
                                }
                    }
                },
                ()->{
                    return Storage.state == Storage.State.RESET;
                },
                new Node[]{spike2, gate, loading, gate, spike1, park}
        );
        gate.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(gatePos[Math.min(gate.index, gatePos.length-1)]);
                },
                ()->{
                    if (gate.index == 0 && chassis.inPosition(80,80,0.2)){
                        timer.reset();
                        return true;
                    }
                    if (gate.index == 1 && (Storage.state == Storage.State.TRANSFER || timer.seconds()>2.5)){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{gate,shoot}
        );
        loading.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(loadingPos);
                },
                ()->{
                    if (chassis.inPosition(40,40,0.2)){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        spike1.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(spike1Pos[Math.min(spike1.index, spike1Pos.length-1)]);
                },
                ()->{
                    return chassis.inPosition(40,40,0.1);
                },
                new Node[]{spike1,shoot}
        );
        spike2.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(spike2Pos[Math.min(spike2.index, spike2Pos.length-1)]);
                },
                ()->{
                    return chassis.inPosition(40,40,0.1);
                },
                new Node[]{spike2,shoot}
        );
        park.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(parkPos);
                },
                ()->{
                    return chassis.inPosition(60,60,0.2);
                },
                new Node[]{park}
        );
    }
    public void update(){
        currentNode.run();
        odo.update();
        chassis.update();
        intake.update();
        shooter.update();
        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];
    }

}
