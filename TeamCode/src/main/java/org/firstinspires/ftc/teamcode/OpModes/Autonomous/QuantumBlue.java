package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Components.Shooter.Hood;
import org.firstinspires.ftc.teamcode.Components.Shooter.Turret;

@Autonomous
public class QuantumBlue extends LinearOpMode {
    org.firstinspires.ftc.teamcode.LogicNodes.QuantumBlue quantumBlue;
    @Override
    public void runOpMode(){
        Chassis.stop = false;
        quantumBlue = new org.firstinspires.ftc.teamcode.LogicNodes.QuantumBlue(hardwareMap);
        Turret.offsetY = -10;
        Hood.offset = -0.0035;
        while (opModeInInit()){
            quantumBlue.intake.update();
            quantumBlue.timer.reset();
            quantumBlue.odo.update();
            quantumBlue.shooter.turret.update();
            quantumBlue.globalTimer.reset();
        }
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            quantumBlue.update();
            if (quantumBlue.globalTimer.seconds()>30.3) requestOpModeStop();
        }
    }
}
