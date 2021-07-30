package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.FoundationGripperOperation;
import org.firstinspires.ftc.teamcode.robot.operations.Operation;
import org.firstinspires.ftc.teamcode.robot.operations.StrafeLeftForDistanceOperation;
import org.firstinspires.ftc.teamcode.robot.operations.WaitOperation;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.ArrayList;
import java.util.Date;


@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Fill&Go", group="Phoebe")
//@Disabled
public class FillAndGo extends LinearOpMode {

    public static final double MOVEMENT_INCREMENT = 3*Field.MM_PER_INCH;
    public static final double SPEED = 0.5;
    @Override
    public void runOpMode() {
        Match match = Match.getNewInstance();
        try {
            match.init();
            Robot robot = match.getRobot();
            robot.init(hardwareMap, this.telemetry, match, Alliance.Color.BLUE);

            ArrayList<Operation> operations = new ArrayList<>();
            Date lastRecorded = new Date();

            //wait to begin - but keeping adding operations
            while (!opModeIsActive() && !isStopRequested()) {
                telemetry.addData("status", "Add operations. Reposition and start when ready");
                telemetry.update();
                if (new Date().getTime() - lastRecorded.getTime() > 500) {
                    boolean somethingAdded = false;
                    double multiplier = 1;
                    if (gamepad1.left_trigger > 0) {
                        multiplier *=2;
                    }
                    if (gamepad1.right_trigger > 0) {
                        multiplier *=2;
                    }
                    double movementToApply = MOVEMENT_INCREMENT * multiplier;

                    if (gamepad1.dpad_up) {
                        operations.add(new DriveForDistanceOperation(movementToApply, SPEED, "Move forward"));
                        if (gamepad1.right_trigger > 0) {
                            robot.queuePrimaryOperation(new DriveForDistanceOperation(movementToApply, SPEED, "Move forward"));
                        }
                        lastRecorded = new Date();
                    } else if (gamepad1.dpad_down) {
                        operations.add(new DriveForDistanceOperation(-movementToApply, SPEED, "Move backward"));
                        if (gamepad1.right_trigger > 0) {
                            robot.queuePrimaryOperation(new DriveForDistanceOperation(-movementToApply, SPEED, "Move backward"));
                        }
                        lastRecorded = new Date();
                    } else if (gamepad1.dpad_left) {
                        operations.add(new StrafeLeftForDistanceOperation(movementToApply, SPEED, "Move left"));
                        if (gamepad1.right_trigger > 0) {
                            robot.queuePrimaryOperation(new StrafeLeftForDistanceOperation(movementToApply, SPEED, "Move left"));
                        }
                        lastRecorded = new Date();
                    } else if (gamepad1.dpad_right) {
                        operations.add(new StrafeLeftForDistanceOperation(-movementToApply, SPEED, "Move right"));
                        if (gamepad1.right_trigger > 0) {
                            robot.queuePrimaryOperation(new StrafeLeftForDistanceOperation(-movementToApply, SPEED, "Move right"));
                        }
                        lastRecorded = new Date();
                    }
                    if (gamepad1.x) {
                        operations.add(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Lower gripper"));
                        if (gamepad1.right_trigger > 0) {
                            robot.queuePrimaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.LOWER, "Lower gripper"));
                        }
                        lastRecorded = new Date();
                    }
                    if (gamepad1.y) {
                        operations.add(new FoundationGripperOperation(FoundationGripperOperation.OperationType.RAISE, "Raise gripper"));
                        if (gamepad1.right_trigger > 0) {
                            robot.queuePrimaryOperation(new FoundationGripperOperation(FoundationGripperOperation.OperationType.RAISE, "Lower gripper"));
                        }
                        lastRecorded = new Date();
                    }
                    if (gamepad1.a) {
                        operations.add(new WaitOperation(1000, "Wait a sec"));
                        lastRecorded = new Date();
                    }
                }
            }

            if (isStopRequested()) {
                return;
            }
            match.setStart();
            Match.log("--------> Starting autonomous");
            for (Operation operation: operations) {
                robot.queuePrimaryOperation(operation);
            }
            while (opModeIsActive() && !isStopRequested()) {

            }
            robot.stop();
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