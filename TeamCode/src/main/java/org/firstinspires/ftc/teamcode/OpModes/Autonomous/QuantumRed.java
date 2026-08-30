package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;

@Autonomous
public class QuantumRed extends LinearOpMode {
    org.firstinspires.ftc.teamcode.LogicNodes.QuantumRed quantumRed;
    @Override
    public void runOpMode(){
        Chassis.stop = false;
        quantumRed = new org.firstinspires.ftc.teamcode.LogicNodes.QuantumRed(hardwareMap);
        Hood.offset = -0.0035;
        while (opModeInInit()){
            quantumRed.intake.update();
            quantumRed.timer.reset();
            quantumRed.odo.update();
            quantumRed.shooter.turret.update();
            quantumRed.globalTimer.reset();
        }
        waitForStart();
        while (opModeIsActive()){
            quantumRed.update();
            if (quantumRed.globalTimer.seconds()>30.3) requestOpModeStop();

        }
    }
}
