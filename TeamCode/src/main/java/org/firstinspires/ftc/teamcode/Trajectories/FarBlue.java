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
import org.firstinspires.ftc.teamcode.Wrappers.LimeLight;

public class FarBlue {

    public static Pose2D shootPos = new Pose2D(-3420, -235, Math.toRadians(165));
    public static Pose2D loadingPos = new Pose2D(-2440, -10, Math.PI);
    public static Pose2D[] spike3Pos = {
            new Pose2D(-3260, -790, Math.PI),
            new Pose2D(-2600, -790, Math.PI),
    };
    public static Pose2D startPos = new Pose2D(-3245,-60,Math.PI);
    public static Pose2D tunnelPos = new Pose2D(-2465, -705, Math.toRadians(165));
    public static Pose2D parkPos = new Pose2D(-3375, -515, Math.toRadians(165));
}