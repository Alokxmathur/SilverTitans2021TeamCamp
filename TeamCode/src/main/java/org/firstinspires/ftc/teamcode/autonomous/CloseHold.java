package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;

import java.io.PrintWriter;
import java.io.StringWriter;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Utility: CloseGripper", group="Phoebe")
//@Disabled
public class CloseHold extends LinearOpMode {
    @Override
    public void runOpMode() {
        try {
            PickerArm pickerArm = new PickerArm(hardwareMap, telemetry);
            pickerArm.closeGripper();
            //wait to begin
            waitForStart();
            while (opModeIsActive() && !isStopRequested()) {
            }
            Match.log("Stopped autonomous  <-------------");
        } catch (Throwable e) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            e.printStackTrace(printWriter);
            telemetry.addLine(stringWriter.toString());
            telemetry.update();
            try {
                Thread.sleep(20000);
            } catch (InterruptedException ex) {
            }
        }
    }
}