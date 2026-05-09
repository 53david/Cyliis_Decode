package org.firstinspires.ftc.teamcode.Trajectories;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class FarRed {
    ElapsedTime timer;
    public Pose2D shootPos = new Pose2D(0,0,0);
    public Pose2D loadingPos = new Pose2D(0,0,0);
    public Pose2D[] spike3Pos = {
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public Pose2D[] tunnelPos = {
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
            new Pose2D(0,0,0),
    };
    public Odo odo;
    public Shooter shooter;
    public Intake intake;
    public Storage storage;
    public Chassis chassis;
    Node shoot,spike3,loading,tunnel;
    public Node currentNode;
    public FarRed(HardwareMap hardwareMap){
        Initializer.start(hardwareMap);
        timer = new ElapsedTime();
        timer.startTime();
        timer.reset();
        Shooter.state = Shooter.State.SHOOT;
        Turret.allienceState = Turret.AllianceState.RED;
        chassis = new Chassis(Chassis.State.PID);
        storage = new Storage();
        intake = new Intake();
        shooter = new Shooter();
        odo = new Odo();
        shoot = new Node("shoot");
        spike3 = new Node("spike3");
        loading = new Node("loading");
        tunnel = new Node("tunnel");
        currentNode = shoot;
        shoot.addConditions(
                ()->{
                    if ((Intake.state == Intake.State.INTAKE || Intake.state == Intake.State.REVERSE)
                            && !storage.IsStorageSpinning()){
                        Intake.state = Intake.State.IDLE;
                    }
                    Shooter.state = Shooter.State.SHOOT;
                    chassis.setTargetPosition(shootPos);
                    if (chassis.inPosition(60,60,0.25) && Storage.state == Storage.State.TRANSFER){
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
                new Node[]{spike3,loading,tunnel,loading,tunnel,loading}
        );
        spike3.addConditions(
                ()->{
                    if (Storage.state != Storage.State.TRANSFER) {
                        Intake.state = Intake.State.INTAKE;
                    }
                    else {
                        Intake.state = Intake.State.REVERSE;
                    }
                    Shooter.state = Shooter.State.IDLE;
                    chassis.setTargetPosition(spike3Pos[Math.min(spike3.index, spike3Pos.length-1)]);
                },
                ()->{
                    if (Storage.state == Storage.State.TRANSFER || timer.seconds()>5.5){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        tunnel.addConditions(
                ()->{
                    if (Storage.state != Storage.State.TRANSFER) {
                        Intake.state = Intake.State.INTAKE;
                    }
                    else {
                        Intake.state = Intake.State.REVERSE;
                    }
                    Shooter.state = Shooter.State.IDLE;
                    chassis.setTargetPosition(spike3Pos[Math.min(tunnel.index, tunnelPos.length-1)]);
                },
                ()->{
                    if (Storage.state == Storage.State.TRANSFER || timer.seconds()>5.5){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
        loading.addConditions(
                ()->{
                    if (Storage.state != Storage.State.TRANSFER) {
                        Intake.state = Intake.State.INTAKE;
                    }
                    else {
                        Intake.state = Intake.State.REVERSE;
                    }
                    Shooter.state = Shooter.State.IDLE;
                    chassis.setTargetPosition(loadingPos);
                },
                ()->{
                    if (Storage.state == Storage.State.TRANSFER || timer.seconds()>5.5){
                        timer.reset();
                        return true;
                    }
                    return false;
                },
                new Node[]{shoot}
        );
    }
    public void update(){
        odo.update();
        shooter.update();
        intake.update();
        chassis.update();
        currentNode.run();
        if (currentNode.transition()){
            currentNode = currentNode.next[Math.min(currentNode.index++,currentNode.next.length-1)];
        }
    }

}
