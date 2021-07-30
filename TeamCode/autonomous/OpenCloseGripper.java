package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.io.PrintWriter;
import java.io.StringWriter;

/**
 * Created by Silver Titans on 09/28/2019
 * <p>
 * Here's our approach for the autonomous period
 * Steps
 * 1. Find Sky-stone in top section of quarry
 * 2. Approach top sky-stone
 * 3. Capture sky-stone
 * 3. Navigate to foundation
 * 5. Place stone
 * 6. Approach bottom sky-stone
 * 7. Capture second sky-stone
 * 8. Navigate to foundation
 * 9. Place stone
 * 10. Move foundation
 * 11. Park
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Utility: Close Gripper", group="Phoebe")
//@Disabled
public class OpenCloseGripper extends LinearOpMode {

    @Override
    public void runOpMode() {
         try {
             Robot robot = new Robot();
             telemetry.addData("Ready:", "Press run to close gripper");
             telemetry.update();
             waitForStart();
             robot.initPickerArm();
         }
         catch (Throwable e) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            e.printStackTrace(printWriter);
            telemetry.addLine(stringWriter.toString());
            telemetry.update();
            Match.log(stringWriter.toString());
            try {
                Thread.sleep(20000);
            } catch (InterruptedException ex) {
            }
        }
    }
}