package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.LogicNodes.CloseBlue;
import org.firstinspires.ftc.teamcode.Wrappers.Initializer;

@Autonomous
public class BlueClose extends LinearOpMode {
    public CloseBlue closeBlue;
    @Override
    public void runOpMode() throws InterruptedException{
        Initializer.isAutonomousActive = true;
        closeBlue = new CloseBlue(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            closeBlue.update();
        }
    }
}
