package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;

import org.firstinspires.ftc.teamcode.LogicNodes.FarBlue;


@Autonomous
public class BlueFar extends LinearOpMode {
    FarBlue farBlue;
    @Override
    public void runOpMode(){
        isAutonomousActive = true;
        farBlue = new FarBlue(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            farBlue.update();
        }
    }
}
