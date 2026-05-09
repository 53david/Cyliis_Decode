package org.firstinspires.ftc.teamcode.Trajectories;

import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class FarBlue {
    Storage storage;
    public Odo odo;
    public Chassis chassis;
    public Intake intake;
    public Shooter shooter;
    public Pose2D shootPos = new Pose2D(0,0,Math.PI);
    public Pose2D[] spike1Pos ={
            new Pose2D(0,0,Math.PI),
            new Pose2D(0,0,Math.PI),
            new Pose2D(0,0,Math.PI),
    };
    public Pose2D loadingZonePos = new Pose2D(0,0,Math.PI);
    Node shoot,spike1,loadingZone;
    public Node currentNode;
    boolean isShootReady = false;
    public FarBlue(HardwareMap hardwareMap){
        Initializer.start(hardwareMap);
        storage = new Storage();
        odo = new Odo();
        chassis = new Chassis(Chassis.State.PID);
        intake = new Intake();
        shooter = new Shooter();
        Storage.state = Storage.State.TRANSFER;
        Turret.allienceState = Turret.AllianceState.BLUE;
        shoot = new Node("shoot");
        spike1 = new Node("spike1");
        loadingZone = new Node("loadingZone");
        currentNode = shoot;
        shoot.addConditions(
                ()->{
                    if (Intake.state == Intake.State.INTAKE && !storage.IsStorageSpinning()){
                        Intake.state = Intake.State.IDLE;
                    }
                    chassis.setTargetSpecialPosition(shootPos);
                    Shooter.state = Shooter.State.SHOOT;
                    if (chassis.inPosition(40,40,0.13) && Math.abs(Initializer.pp.getVelX(DistanceUnit.MM))<=25
                            && Math.abs(Initializer.pp.getVelY(DistanceUnit.MM))<=25 && !isShootReady){
                        Storage.state = Storage.State.SHOOT;
                        isShootReady = true;
                    }

                },
                ()->{
                    return Storage.state == Storage.State.RESET;
                },
                new Node[]{loadingZone,spike1,loadingZone,loadingZone}
        );
        spike1.addConditions(
                ()->{
                    isShootReady = false;
                    Intake.state = Intake.State.INTAKE;
                    Shooter.state = Shooter.State.IDLE;
                    chassis.setTargetPosition(spike1Pos[Math.min(spike1.index, spike1Pos.length-1)]);
                },
                ()->{
                    return chassis.inPosition(60,60,0.2);
                },
                new Node[]{shoot}
        );
        loadingZone.addConditions(
                ()->{
                    isShootReady = false;
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(loadingZonePos);
                },
                ()->{
                    return chassis.inPosition(40,40,0.15);
                },
                new Node[]{shoot}

        );
    }
    public void update(){
        odo.update();
        chassis.update();
        intake.update();
        shooter.update();
        if (currentNode.transition()){
            currentNode = currentNode.next[Math.min(currentNode.index++,currentNode.next.length-1)];
        }
    }
}
