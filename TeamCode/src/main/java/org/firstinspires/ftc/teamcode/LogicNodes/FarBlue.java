package org.firstinspires.ftc.teamcode.LogicNodes;

import static org.firstinspires.ftc.teamcode.Trajectories.FarBlue.loadingPos;
import static org.firstinspires.ftc.teamcode.Trajectories.FarBlue.parkPos;
import static org.firstinspires.ftc.teamcode.Trajectories.FarBlue.shootPos;
import static org.firstinspires.ftc.teamcode.Trajectories.FarBlue.spike3Pos;
import static org.firstinspires.ftc.teamcode.Trajectories.FarBlue.tunnelPos;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Latch;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Trajectories.CloseBlue;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

public class FarBlue {
    ElapsedTime timer;
    public Odo odo;
    public Shooter shooter;
    public Intake intake;
    public Storage storage;
    public Chassis chassis;
    Node shoot,spike3,loading,tunnel,park;
    public Node currentNode;
    public FarBlue(HardwareMap hardwareMap){
        Initializer.start(hardwareMap);
        timer = new ElapsedTime();
        timer.startTime();
        timer.reset();
        Storage.state = Storage.State.TRANSFER;
        Shooter.state = Shooter.State.SHOOT;
        chassis = new Chassis(Chassis.State.PID);
        storage = new Storage();
        intake = new Intake();
        shooter = new Shooter(Shooter.State.SHOOT);
        odo = new Odo();
        odo.reset();
        shoot = new Node("shoot");
        spike3 = new Node("spike3");
        loading = new Node("loading");
        tunnel = new Node("tunnel");
        park = new Node("park");
        currentNode = shoot;
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


