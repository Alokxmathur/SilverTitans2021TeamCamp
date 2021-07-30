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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.components.vision.OpenCVWebcam;
import org.opencv.core.Scalar;

import java.util.Locale;

@TeleOp(name = "Phoebe: Wobble Finder", group = "Phoebe")

public class WobbleFinderTeleOp extends LinearOpMode {
    OpenCVWebcam webcam;

    @Override
    public void runOpMode() {
        Servo cameraServo = hardwareMap.get(Servo.class, "cameraServo");
        cameraServo.setPosition(OpenCVWebcam.CAMERA_SERVO_FORWARD_POSITION);

        webcam = new OpenCVWebcam();
        webcam.setMaxX(448);
        webcam.setMaxY(800);
        webcam.init(hardwareMap, telemetry, OpenCVWebcam.WOBBLE_COLOR_MIN, OpenCVWebcam.WOBBLE_COLOR_MAX);

        waitForStart();

        while (opModeIsActive()) {
            Scalar mean = webcam.getMean();
            telemetry.addData("Wobble location",
                    String.format(Locale.getDefault(), "(%d,%d) - (%d,%d)", webcam.getMinX(), webcam.getMinY(), webcam.getMaxX(), webcam.getMaxY()));
            telemetry.addData("Mean", mean == null ? "-" : mean.toString());
            telemetry.addData("Bounds", webcam.getBounds());

            telemetry.update();
            if (gamepad1.left_stick_x < 0) {
                webcam.decrementMinX();
            }
            if (gamepad1.left_stick_x > 0) {
                webcam.incrementMinX();
            }
            if (gamepad1.right_stick_x < 0) {
                webcam.decrementMaxX();
            }
            if (gamepad1.right_stick_x > 0) {
                webcam.incrementMaxX();
            }

            if (gamepad1.left_stick_y < 0) {
                webcam.decrementMinY();
            }
            if (gamepad1.left_stick_y > 0) {
                webcam.incrementMinY();
            }
            if (gamepad1.right_stick_y < 0) {
                webcam.decrementMaxY();
            }
            if (gamepad1.right_stick_y > 0) {
                webcam.incrementMaxY();
            }


            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }
}