package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.components.PickerArm;

import java.io.PrintWriter;
import java.io.StringWriter;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Utility: Open/close capstone", group="Phoebe")
//@Disabled
public class OpenCloseCapstone extends LinearOpMode {

    @Override
    public void runOpMode() {
        Match match = Match.getNewInstance();
        try {
            PickerArm pickerArm = new PickerArm(hardwareMap, telemetry);
            pickerArm.releaseCapstone();
            //wait to begin
            waitForStart();
            pickerArm.holdCapStone();
            while (opModeIsActive() && !isStopRequested()) {
            }
            Match.log("Stopped autonomous  <-------------");
        } catch (Throwable e) {
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