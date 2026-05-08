package org.firstinspires.ftc.teamcode.Trajectories;

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
    public static Pose2D[] beforeSpike2Pos = {
            new Pose2D(-1910, -100, Math.PI/2) ,
            new Pose2D(-2450, -100, Math.PI/2)
    };

    public static Pose2D[] spike2Pos = {
            new Pose2D(-1910, 765, Math.PI/2),
            new Pose2D(-2450, 765, Math.PI/2)
    };
    public static Pose2D beforeShootPos = new Pose2D(-1930, 600, Math.PI/2-0.3);
    public static Pose2D beforeGatePos = new Pose2D(-1845, 620, Math.PI/2);
    public static Pose2D gatePos = new Pose2D(-1875, 700, Math.PI/2+0.4);

    public Pose2D beforeSpike1Pos = new Pose2D(-1270 , -450 , Math.PI/2);
    public Pose2D spike1Pos = new Pose2D(-1270 , 620 , Math.PI/2);
    Node shoot,beforeSpike2,beforeSpike1,openGate,spike1,spike2,afterCollecting,beforeGate,gate;
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
        beforeSpike2 = new Node("beforeSpike2");
        beforeSpike1 = new Node("beforeSpike1");
        openGate = new Node("openGate");
        spike1 = new Node("spike1");
        spike2 = new Node("spike2");
        afterCollecting = new Node("afterCollecting");
        beforeGate = new Node("beforeGate");
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
                new Node[]{beforeSpike2, beforeGate, beforeGate, beforeSpike1}
        );
        beforeSpike2.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.IDLE;
                    chassis.setTargetPosition(beforeSpike2Pos[Math.min(beforeSpike2.index , beforeSpike2Pos.length-1) ] );
                },
                ()->{
                    return chassis.inPosition(35, 35, 0.1);
                },
                new Node[]{spike2}
        );
        spike2.addConditions(
                ()->{
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(spike2Pos[Math.min(spike2.index, spike2Pos.length-1)]);
                },
                ()->{
                    return chassis.inPosition(35, 35, 0.1);
                },
                new Node[]{afterCollecting}
        );
        afterCollecting.addConditions(
                ()->{
                    chassis.setTargetPosition(beforeShootPos);
                    Shooter.state = Shooter.State.SHOOT;
                    if (Storage.state == Storage.State.TRANSFER){
                        Intake.state = Intake.State.REVERSE;
                    }
                },
                ()->{
                    return chassis.inPosition(35, 35, 0.1);
                },
                new Node[]{shoot}
        );
        beforeGate.addConditions(
                ()->{
                    chassis.setTargetPosition(beforeGatePos);
                    Shooter.state = Shooter.State.IDLE;
                    timer.reset();
                },
                ()->{
                    return chassis.inPosition(60, 60, 0.15);
                },
                new Node[]{gate}
        );
        gate.addConditions(
                ()->{
                    chassis.setTargetPosition(gatePos);
                    Intake.state = Intake.State.INTAKE;
                    Shooter.state = Shooter.State.IDLE;
                },
                ()->{
                    return Storage.state == Storage.State.TRANSFER || timer.seconds()>5.5;
                },
                new Node[]{afterCollecting}
        );
        beforeSpike1.addConditions(
                ()->{
                    chassis.setTargetPosition(beforeSpike1Pos);
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.IDLE;
                },
                ()->{
                    return chassis.inPosition(40, 40, 0.1);
                },
                new Node[]{spike1}
        );
        spike1.addConditions(
                ()->{
                    chassis.setTargetPosition(spike1Pos);
                    Intake.state = Intake.State.INTAKE;
                    Shooter.state = Shooter.State.IDLE;
                },
                ()->{
                    return chassis.inPosition(100, 100, 0.1);
                },
                new Node[]{shoot}
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
