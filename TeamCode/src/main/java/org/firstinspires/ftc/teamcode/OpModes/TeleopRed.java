package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

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

@TeleOp(name = "We Ioan")
public class TeleopRed extends LinearOpMode {

    Intake intake;
    Chassis drive;
    Shooter shooter;
    Odo odo;
    VoltageSensor voltageSensor;
    @Override
    public void runOpMode() {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        isAutonomousActive = false;
        Initializer.start(hardwareMap);
        odo = new Odo();
        intake =new Intake();
        drive =new Chassis(Chassis.State.DRIVE);
        shooter =new Shooter(Shooter.State.SHOOT);
        Turret.allianceState = Turret.AllianceState.RED;
        Hood.state = Hood.State.IDLE;
        waitForStart();
        while (opModeIsActive()) {
            gm1.copy(gamepad1);
            gm2.copy(gamepad2);
            intake.update();
            shooter.update();
            drive.update();
            odo.update();
            prevgm1.copy(gm1);
            prevgm2.copy(gm2);

            if (gamepad1.psWasPressed()){
                odo.reset();
            }
            if (Storage.isTransferReady){
                gamepad1.rumble(200);
            }
            telemetry.addData("ALLIANCE",Turret.allianceState);
            telemetry.addData("X",Odo.getX());
            telemetry.addData("Y",Odo.getY());
            telemetry.addData("Heading",Odo.getHeading());
            telemetry.addData("Intake state", Storage.state);
            telemetry.addData("Shooter state", Shooter.state);
            telemetry.addData("Flywheel velocity", FlyWheel.getVelocity());
            if (ColorDetection.state == ColorDetection.State.SORT) {
                telemetry.addData("Ball1", ColorDetection.ball1);
                telemetry.addData("Ball2", ColorDetection.ball2);
                telemetry.addData("Ball3", ColorDetection.ball3);
            }
            if (LimeLight.isActive()) {
                telemetry.addData("Target heading", LimeLight.getHeading());
                telemetry.addData("Distance", LimeLight.getDistance());
                telemetry.addData("Area", LimeLight.getArea());
            } else {
                telemetry.addLine("Waiting for stream..");

            }
            telemetryM.addData("Is object nearby?",ColorDetection.isBallInStorage());
            telemetryM.addData("Is storage spinning",Storage.IsStorageSpinning());
            telemetryM.addData("Target pos",Math.toDegrees(Storage.target));
            telemetryM.addData("Voltage",voltageSensor.getVoltage());
            telemetryM.addData("Flywheel velocity", FlyWheel.getVelocity());
            telemetryM.addData("Target Velocity",ShooterCalculator.vel);
            telemetryM.addData("Hood Angle",ShooterCalculator.hoodPos);
            telemetryM.update();
            telemetry.update();
        }
    }
}