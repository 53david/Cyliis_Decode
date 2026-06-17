package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Wrappers.Initializer.isAutonomousActive;
import org.firstinspires.ftc.teamcode.Trajectories.CloseBlue;

@Autonomous
public class BlueClose extends LinearOpMode {
    CloseBlue closeBlue;
    boolean ok = true;
    @Override
    public void runOpMode() throws InterruptedException{
        isAutonomousActive = true;
        closeBlue = new CloseBlue(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            closeBlue.update();
        }
    }

}
