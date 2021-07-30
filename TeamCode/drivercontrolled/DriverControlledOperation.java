/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drivercontrolled;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.operations.CameraOperation;

import java.io.PrintWriter;
import java.io.StringWriter;
import java.util.Date;

/**
 * This file provides basic Telop driving for a Phoebe robot.
 * The code is structured as an Iterative OpMode
 * <p>
 * This OpMode uses the common Phoebe hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePhoebe class.
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for a Phoebe
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Phoebe: Driver Controlled", group = "Phoebe")
//@Disabled
public class DriverControlledOperation extends OpMode {

    {
        msStuckDetectInit     = 20000;
    }

    private Robot robot = Match.getInstance().getRobot();
    private float lastPower = 0;
    private float lastTurn = 0;
    private float lastLatchPower = 0;

    private Match match;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        this.match = Match.getInstance();
        //robot.initCamera(match.getAllianceColor());
        match.setTeleopStartTime(new Date());

        try {
            robot.setState("Initialized");
            Match.getInstance().updateTelemetry(telemetry, "Press start now");
        } catch (Throwable e) {
            StringWriter stringWriter = new StringWriter();
            PrintWriter printWriter = new PrintWriter(stringWriter);
            e.printStackTrace(printWriter);
            telemetry.addLine(stringWriter.toString());
            telemetry.update();
            RobotLog.ee("Silver Titans", e, "Exception");
            try {
                Thread.sleep(1000);
            } catch (InterruptedException ex) {
            }
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (robot.fullyInitialized()) {
            try {
                robot.handleGameControllers(gamepad1, gamepad2);
                match.updateTelemetry(telemetry, robot.getState());
            } catch (Throwable e) {
                StringWriter stringWriter = new StringWriter();
                PrintWriter printWriter = new PrintWriter(stringWriter);
                e.printStackTrace(printWriter);
                telemetry.addLine(stringWriter.toString());
                telemetry.update();
                RobotLog.e("TeleOp run", e, "Error");
            }
        }
    }

    @Override
    public void start() {
        robot.queuePrimaryOperation(new CameraOperation(CameraOperation.CameraOperationType.FLASH_ON, "Turn on flash"));
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        if (robot.fullyInitialized()) {
            robot.stop();
        }
    }
}
