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

    public static Pose2D shootPos = new Pose2D(-25, 950, Math.PI/2*3);
    public static Pose2D loadingPos = new Pose2D(-55, 60, Math.PI/2*3);
    public static Pose2D[] spike3Pos = {
            new Pose2D(-720, -880, Math.PI/2*3),
            new Pose2D(-720, -250, Math.PI/2*3),
    };
    public static Pose2D tunnelPos = new Pose2D(-435, 60, Math.PI/2*3);
    public static Pose2D parkPos = new Pose2D(-275, 815, Math.PI/2*3);
}