package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.gm2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm1;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.prevgm2;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.telemetryM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Intake.ColorDetection;
import org.firstinspires.ftc.teamcode.Components.Intake.Intake;
import org.firstinspires.ftc.teamcode.Components.Intake.Storage;
import org.firstinspires.ftc.teamcode.Components.Shooter.FlyWheel;
import org.firstinspires.ftc.teamcode.Components.Shooter.Shooter;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Vision;

@TeleOp
public class TeleopRed extends LinearOpMode {
    Chassis drive;
    Shooter shooter;
    Intake intake;
    Odo odo;
    Vision vision;
    @Override
    public void runOpMode(){
        isAutonomousActive = false;
        Initializer.start(hardwareMap);
        drive = new Chassis(Chassis.State.DRIVE);
        shooter = new Shooter();
        intake = new Intake();
        odo = new Odo();
        Turret.allienceState = Turret.AllianceState.RED;
        Shooter.state = Shooter.State.SHOOT;
        waitForStart();
        while (opModeIsActive()){
            gm1.copy(gamepad1);
            gm2.copy(gamepad2);
            drive.update();
            shooter.update();
            vision.update();
            intake.update();
            odo.update();
            prevgm1.copy(gm1);
            prevgm2.copy(gm2);

            if (gamepad1.psWasPressed()){
                odo.reset();
            }
            if (Storage.isTransferReady) {
                gamepad1.rumble(200);
            }
            telemetry.addData("ALLIANCE",Turret.allienceState);
            telemetry.addData("X",Odo.getX());
            telemetry.addData("Y",Odo.getY());
            telemetry.addData("Heading",Odo.getHeading());
            telemetryM.addData("Distance",Odo.distance());
            telemetry.addData("Intake state", Storage.state);
            telemetry.addData("Shooter state", Shooter.state);
            telemetry.addData("Flywheel velocity", FlyWheel.getVelocity());
            telemetry.addData("Ball1", ColorDetection.ball1);
            telemetry.addData("Ball2",ColorDetection.ball2);
            telemetry.addData("Ball3",ColorDetection.ball3);
            if (Vision.isActive()) {
                telemetryM.addData("Target heading", Vision.getHeading());
                telemetryM.addData("Distance", Vision.getDistance());
                telemetryM.addData("Area", Vision.getArea());
            } else {
                telemetryM.addLine("Waiting for stream..");

            }
            telemetryM.update();
        }
    }
}
