/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drivercontrolled;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousHelper;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.firstinspires.ftc.teamcode.robot.operations.BearingOperation;
import org.firstinspires.ftc.teamcode.robot.operations.DistanceInDirectionOperation;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ShootingOperation;

@TeleOp(name = "Phoebe: Rings Finder", group = "Phoebe")

public class RingFinderTeleOp extends DriverControlledOperation {
    OpenCVWebcam webcam;
    @Override
    public void init() {
        super.init();
        webcam = robot.getWebcam();
        webcam.init(hardwareMap, telemetry, OpenCVWebcam.RING_COLOR_MIN, OpenCVWebcam.RING_COLOR_MAX);
    }
    @Override
    public void loop() {
        telemetry.addData("Rings", robot.getNumberOfRings());
        if (webcam.seeingRing()) {
            telemetry.addData("Distance", webcam.getDistanceToObjectFromCamera());
            telemetry.addData("Focal Length (at 24 inches)", (webcam.getMaxY() - webcam.getMinY()) * 24 / 5);
            telemetry.addData("Camera-Ring position", webcam.getRelativeRingPosition());
            telemetry.addData("Center-position", webcam.getDistanceToObjectFromCenter()
                    + "@" + Math.toDegrees(webcam.getAngleToObjectFromCenter()));
        }
        telemetry.addData("Robot position", robot.getPosition());
        telemetry.addData("Bounds", webcam.getBounds());

        telemetry.update();
        if (gamepad2.left_stick_x < 0) {
            webcam.decrementMinX();
        }
        if (gamepad2.left_stick_x > 0) {
            webcam.incrementMinX();
        }
        if (gamepad2.right_stick_x < 0) {
            webcam.decrementMaxX();
        }
        if (gamepad2.right_stick_x > 0) {
            webcam.incrementMaxX();
        }

        if (gamepad2.left_stick_y < 0) {
            webcam.decrementMinY();
        }
        if (gamepad2.left_stick_y > 0) {
            webcam.incrementMinY();
        }
        if (gamepad2.right_stick_y < 0) {
            webcam.decrementMaxY();
        }
        if (gamepad2.right_stick_y > 0) {
            webcam.incrementMaxY();
        }
        if (robot.allOperationsCompleted() && gamepad2.start && webcam.seeingRing())
        {
            robot.queueSecondaryOperation(new IntakeOperation(IntakeOperation.IntakeOperationType.START_INTAKE, "Start intake"));
            robot.queueTertiaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.INTAKE_POSITION, "Assume intake position"));
            double bearing = Math.toDegrees(robot.getCurrentTheta() - webcam.getAngleToObjectFromCenter());
            robot.queuePrimaryOperation(new BearingOperation(bearing, "Turn to ring @"
                    + Math.toDegrees(webcam.getAngleToObjectFromCamera())));
            robot.queuePrimaryOperation(new DistanceInDirectionOperation(
                    webcam.getDistanceToObjectFromCenter()* Field.MM_PER_INCH,
                    bearing,
                    AutonomousHelper.REGULAR_SPEED,
                    "Consume ring"));
            //robot.queuePrimaryOperation(new DriveToPositionOperation(webcam.getAbsoluteRingPosition(robot.getPose()), "Intake ring"));
        }
        robot.handleDriveTrain(gamepad1);
        robot.handleShooter(gamepad1);
        robot.handleIntake(gamepad1, gamepad2);
    }

}