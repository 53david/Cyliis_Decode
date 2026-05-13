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

public class FarBlue {
    Storage storage;
    public Odo odo;
    public Chassis chassis;
    public Intake intake;
    public Shooter shooter;
    public Pose2D shootPos = new Pose2D(0,0,Math.PI);
    public Pose2D[] spike3Pos ={
            new Pose2D(0,0,Math.PI),
            new Pose2D(0,0,Math.PI),
    };
    public Pose2D[] tunnelPos={
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public Pose2D parkPos= new Pose2D(0,0,0);
    public Pose2D loadingPos = new Pose2D(0,0,Math.PI);
    Node shoot,spike3,tunnel,loading,park;
    public Node currentNode;
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
        spike3 = new Node("spike3");
        loading = new Node("loading");
        tunnel = new Node("tunnel");
        park = new Node("park");
        currentNode = shoot;
        shoot.addConditions(
                ()->{
                    if (Intake.state == Intake.State.INTAKE && !storage.IsStorageSpinning()){
                        Intake.state = Intake.State.IDLE;
                    }
                    chassis.setTargetSpecialPosition(shootPos);
                    Shooter.state = Shooter.State.SHOOT;
                    if (chassis.inPosition(40,40,0.13) && Math.abs(Initializer.pp.getVelX(DistanceUnit.MM))<=25
                            && Math.abs(Initializer.pp.getVelY(DistanceUnit.MM))<=25){
                        Storage.state = Storage.State.SHOOT;
                    }

                },
                ()->{
                    return Storage.state == Storage.State.RESET;
                },
                new Node[]{spike3,loading,tunnel,loading,tunnel,loading,park}
        );
        spike3.addConditions(
                ()->{
                    Intake.state = Intake.State.INTAKE;
                    Shooter.state = Shooter.State.IDLE;
                    chassis.setTargetPosition(spike3Pos[Math.min(spike3.index, spike3Pos.length-1)]);
                },
                ()->{
                    return chassis.inPosition(60,60,0.2);
                },
                new Node[]{spike3,shoot}
        );
        loading.addConditions(
                ()->{
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(loadingPos);
                },
                ()->{
                    return chassis.inPosition(40,40,0.15);
                },
                new Node[]{shoot}

        );
        tunnel.addConditions(
                ()->{
                    if (Storage.state != Storage.State.TRANSFER){
                        Intake.state = Intake.State.INTAKE;
                    }
                    else {
                        Intake.state = Intake.State.REVERSE;
                    }
                    Shooter.state = Shooter.State.IDLE;
                    Intake.state = Intake.State.INTAKE;
                    chassis.setTargetPosition(tunnelPos[Math.min(tunnel.index, tunnelPos.length-1)]);
                },
                ()->{
                    return chassis.inPosition(60,60,0.2);
                },
                new Node[]{tunnel,shoot}
        );
        park.addConditions(
                ()->{
                    chassis.setTargetPosition(parkPos);
                },
                ()->{
                    return chassis.inPosition(60,60,0.2);
                },
                new Node[]{park}
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
