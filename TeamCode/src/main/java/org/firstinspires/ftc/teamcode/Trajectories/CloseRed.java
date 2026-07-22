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
            new Pose2D(-1600, 725, -Math.PI / 2),
            new Pose2D(-1300, 450, -Math.PI / 3),
            new Pose2D(-1300, 450, -Math.PI / 3),
            new Pose2D(-1300, 450, -Math.PI / 3),
            new Pose2D(-1300, 450, -Math.PI / 4),
            new Pose2D(-1300, 450, -Math.PI / 4),
            new Pose2D(-800, 500, -Math.PI / 6),

    };
    public static Pose2D[] gatePos = {
            new Pose2D(-1720, 560, -Math.PI / 2),
            new Pose2D(-1880, 755, -Math.PI * 2 / 3),

    };
    public static Pose2D parkPos = new Pose2D(-800, -500, -Math.PI / 3);
    public static Pose2D spike1Pos = new Pose2D(-1200, 610, -Math.PI / 2);
    public static Pose2D spike2Pos = new Pose2D(-1950, 760, -Math.PI * 2 / 3);
}