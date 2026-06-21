package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm1;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;




import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.ColorDetection;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;

import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Math.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Wrappers.LimeLight;

@TeleOp
public class TeleopRed extends LinearOpMode {

    Intake intake;
    Chassis drive;
    Shooter shooter;
    LimeLight limeLight;
    Odo odo;
    @Override
    public void runOpMode() {
        isAutonomousActive = false;
        Initializer.start(hardwareMap);
        odo = new Odo();
        intake =new Intake();
        drive =new Chassis(Chassis.State.DRIVE);
        shooter =new Shooter();
        limeLight = new LimeLight(LimeLight.StreamState.CLOSE);
        Turret.allianceState = Turret.AllianceState.RED;
        Shooter.state = Shooter.State.SHOOT;
        Hood.state = Hood.State.IDLE;
        LimeLight.state = LimeLight.State.RED;
        waitForStart();
        while (opModeIsActive()) {
            for (LynxModule hub : Initializer.allHubs) {
                hub.clearBulkCache();
            }
            gm1.copy(gamepad1);
            gm2.copy(gamepad2);
            intake.update();
            drive.update();
            shooter.update();
            odo.update();
            limeLight.update();
            prevgm1.copy(gm1);
            prevgm2.copy(gm2);

            if (gamepad1.psWasPressed()){
                odo.reset();
            }
            if (Storage.isTransferReady) {
                gamepad1.rumble(200);
            }
            if (gamepad1.dpadRightWasPressed() && ColorDetection.state == ColorDetection.State.RAPID){
                ColorDetection.state = ColorDetection.State.SORT;
            }

            if (gamepad1.dpadRightWasPressed() && ColorDetection.state != ColorDetection.State.RAPID){
                ColorDetection.state = ColorDetection.State.RAPID;
            }
            telemetryM.addData("Velocity",FlyWheel.getVelocity());
            telemetryM.addData("Target", ShooterCalculator.fwVel(Odo.distance()));
            telemetryM.addData("Storage target",Math.toDegrees(Storage.target));
            telemetryM.addData("Storage pos",Math.toDegrees(Storage.FromVtoRads()));
            telemetry.addData("ALLIANCE",Turret.allianceState);
            telemetry.addData("X",Odo.getX());
            telemetry.addData("Y",Odo.getY());
            telemetry.addData("Heading",Odo.getHeading());
            telemetryM.addData("Distance",Odo.distance());
            telemetry.addData("Intake state", Storage.state);
            telemetry.addData("Shooter state", Shooter.state);
            telemetryM.addData("Hood Angle",ShooterCalculator.hoodAngle(FlyWheel.getVelocity()));
            telemetry.addData("Flywheel velocity", FlyWheel.getVelocity());
            if (ColorDetection.state == ColorDetection.State.SORT) {
                telemetry.addData("Ball1", ColorDetection.ball1);
                telemetry.addData("Ball2", ColorDetection.ball2);
                telemetry.addData("Ball3", ColorDetection.ball3);
            }
            if (LimeLight.isActive()) {
                telemetryM.addData("Target heading", LimeLight.getHeading());
                telemetryM.addData("Distance", LimeLight.getDistance());
                telemetryM.addData("Area", LimeLight.getArea());
            } else {
                telemetryM.addLine("Waiting for stream..");

            }
            telemetry.update();
        }
    }
}