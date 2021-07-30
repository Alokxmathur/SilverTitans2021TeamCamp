package org.firstinspires.ftc.teamcode.drivercontrolled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

@TeleOp(name="Test T265", group="Phoebe")
public class TestT265Camera extends OpMode
{
    public static final double T265_OFFSET_FRONT = MecanumDriveTrain.DRIVE_TRAIN_LENGTH/1000/2;
    public static final double  T265_CAMERA_OFFSET_LEFT = 0;
    public static final double T265_ROTATION = 180;

    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera t265Camera = null;
    private static Transform2d t265LocationOnRobot =
            new Transform2d(new Translation2d(T265_OFFSET_FRONT, T265_CAMERA_OFFSET_LEFT),
                    new Rotation2d(Math.toRadians(T265_ROTATION)));
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private Match match = Match.getNewInstance();
    private Robot robot = match.getRobot();

    @Override
    public void init() {
        if (t265Camera == null) {
            t265Camera = new T265Camera(t265LocationOnRobot, 0.8, hardwareMap.appContext);
            t265Camera.setPose(new Pose2d(new Translation2d(), new Rotation2d(Math.PI)));
            try {
                t265Camera.start();
            }
            catch (Throwable e) {
                if (!e.getMessage().equals("Camera is already started")) {
                    RobotLog.logStackTrace("SilverTitans: Error starting real sense", e);
                    throw e;
                }
            }
        }
        robot.init(hardwareMap, telemetry, match);
    }

    @Override
    public void init_loop() {
        if (!Field.isInitialized()) {
            telemetry.addData("Status", "Trajectories initializing, please wait");
        }
        else if (robot.fullyInitialized()) {
            telemetry.addData("Status", "Let's go");
        }
        else {
            telemetry.addData("Status", "Waiting for cameras to initialize");
        }
        telemetry.update();
    }

    @Override
    public void start() {
        try {
            t265Camera.start();
        }
        catch (Throwable e) {
            if (!e.getMessage().equals("Camera is already started")) {
                RobotLog.logStackTrace("SilverTitans: Error starting real sense", e);
                throw e;
            }
        }
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = t265Camera.getLastReceivedCameraUpdate();
        if (up == null) {
            field.fillCircle(0, 0, 30);
            return;
        }

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();
        rotation.rotateBy(new Rotation2d(Math.PI));

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);

        robot.handleGameControllers(gamepad1, gamepad2);
    }

    @Override
    public void stop() {
        t265Camera.stop();
    }

}