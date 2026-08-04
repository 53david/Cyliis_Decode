package org.firstinspires.ftc.teamcode.LogicNodes;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Wrappers.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

public class CloseRed {
    Odo odo;
    ElapsedTime timer;
    Chassis chassis;
    Shooter shooter;
    Intake intake;
    Node shoot,spike1,spike2,gate,loading,park,feed;
    public Node currentNode;
    public CloseRed(HardwareMap hardwareMap){
        timer = new ElapsedTime();
        Hardware.init(hardwareMap);
        odo = new Odo();
        chassis = new Chassis(Chassis.State.PID);
        intake = new Intake();
        shooter = new Shooter(Shooter.State.IDLE);
        shoot = new Node("shoot");
        spike1 =  new Node("spike1");
        spike2 = new Node("spike2");
        gate = new Node("gate");
        loading = new Node("loading");
        park = new Node("park");
        feed = new Node("feed");
        currentNode = shoot;

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
