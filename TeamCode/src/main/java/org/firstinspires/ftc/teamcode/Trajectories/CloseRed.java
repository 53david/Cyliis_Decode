package org.firstinspires.ftc.teamcode.Trajectories;

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
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class CloseRed {
    public static Pose2D[] shootPos = {
            new Pose2D(-1325, 350, Math.toRadians(225)),
            new Pose2D(-1425, 420, Math.PI*3/2+0.12),
            new Pose2D(-1425, 420, Math.PI*3/2+0.12),
            new Pose2D(-1425, 420, Math.PI*3/2+0.12),
            new Pose2D(-1425, 420, Math.PI*3/2+0.12),
            new Pose2D(-1425, 420, Math.PI*3/2+0.12),
            new Pose2D(-1425, 420, Math.PI*3/2+0.12),

    };
    public static Pose2D[] gatePos = {
            new Pose2D(-1780, -550, Math.PI*3/2+0.12),
            new Pose2D(-1860, -690, Math.toRadians(238)),

    };
    public static Pose2D feedPos = new Pose2D(-1440, -650, Math.PI*3/2);
    public static Pose2D parkPos = new Pose2D(-800, -500, -Math.PI / 3);
    public static Pose2D[] spike2Pos ={
            new Pose2D(-1900, 50, Math.PI*3/2),
            new Pose2D(-1920, -750, Math.PI*3/2),

    };
    public static Pose2D[] spike1Pos ={
            new Pose2D(-1330, 0, Math.PI*3/2),
            new Pose2D(-1330,  -580, Math.PI*3/2),
    };
}