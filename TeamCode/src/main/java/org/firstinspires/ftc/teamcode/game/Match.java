package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.components.drivetrain.MecanumDriveTrain;

import java.util.Date;

/**
 * Created by Silver Titans on 9/19/17. This is Cameron's comment
 */

public class Match {

    public static final String VUFORIA_KEY =
            "AThl1OD/////AAABmeGI+NPSRE6NuQYkRBmEO3J1J1bQnU4iohBTf5Prkrl/hAPMSgi+Ot0K5qbHfg9uO9KJSa7zI9Bz2JNGsc45xlHpZA2Kr3+ADuKJp8qobRQ9FSxNZzSaIFpZxnzuMaxYU9vAyk51QoRI8NNyCjVy9AsRXVaNnhU2QbwY6ovKMBcC9zh7u2Tdx55It9Sp2haJcO8FEz9I8dtTM0wj3GD/EchSWnP9elAuweQX8fAuxeryn/zLJsIoelt5VCH1L9qgu2wDgvnDd3pnsI5iKC2AJOZVPs9ujLXQjvPbkXdUAGtmUHyCE8ACDoU3a24NitYGkfa8SUfx/hG/qgRv7zD3419aEcJkmL231BJacf1TD/Hc";

    static Match match;
    public static String TEAM = "SilverTitans";
    private Robot robot = null;
    private Field field = null;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Date startTime = new Date();
    private Date teleopStartTime = new Date();
    private Field.RingCount numberOfRings;
    private Alliance.Color allianceColor;

    private Field.StartingPosition startingPosition;

    synchronized public static Match getNewInstance() {
        match = new Match();
        return match;
    }

    synchronized public static Match getInstance() {
        if (match == null) {
            return getNewInstance();
        }
        else {
            return match;
        }
    }

    public void setStart() {
        this.startTime = new Date();
        log("Starting autonomous >>>>>>>>>>>>>");
    }

    /**
     * Return the number of mill-seconds since the match was started
     * @return
     */
    public long getElapsed() {
        return new Date().getTime() - startTime.getTime();
    }

    public Date getTeleopStartTime() {
        return teleopStartTime;
    }

    public void setTeleopStartTime(Date teleopStartTime) {
        this.teleopStartTime = teleopStartTime;
    }

    synchronized public Robot getRobot() {
        if (robot == null) {
            robot = new Robot();
            log ("Created new robot instance");
        }
        return robot;
    }

    public Field getField()
    {
        if (field == null) {
            field = new Field();
            log("Created new field instance");
        }
        return field;
    }

    public void init() {
        robot = new Robot();
        field = new Field();
    }

    public static void log(String s) {
        RobotLog.a(TEAM + ":" + s);
    }

    public void updateTelemetry(Telemetry telemetry, String status) {
        updateTelemetry(telemetry, status, true);
    }
    /**
     * Give the driver station a state of the union
     *
     * @param telemetry
     */
    public void updateTelemetry(Telemetry telemetry, String status, boolean showPosition) {

        if (robot != null && field != null) {
            // Send telemetry message to signify robot context;
            telemetry.addData("State:", status + ", rings: " + getNumberOfRings());
            telemetry.addData("Shooter", robot.getShooterStatus());
            telemetry.addData("Intake", robot.getIntakeStatus());
            if (showPosition) {
                telemetry.addData("Position", robot.getPosition());
            }
            telemetry.addData("TrajectoryError", robot.getLastTrajectoryError());
            telemetry.addData("Arm", robot.getPickerArmStatus());
            updateDashBoard();
        }
        else {
            telemetry.addData("Context", "Robot not initialized");
        }
        telemetry.update();
    }

    public void setAlliance(Alliance.Color allianceColor) {
        this.allianceColor = allianceColor;
    }

    public Alliance.Color getAllianceColor() {
        return allianceColor;
    }

    public Field.RingCount getNumberOfRings() {
        return numberOfRings == null ? Field.RingCount.UNKNOWN : numberOfRings;
    }
    public void setNumberOfRings(Field.RingCount numberOfRings) {
        this.numberOfRings = numberOfRings;
    }

    public Field.StartingPosition getStartingPosition() {
        return startingPosition;
    }

    public void setStartingPosition(Field.StartingPosition startingPosition) {
        this.startingPosition = startingPosition;
    }

    public void updateDashBoard() {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        Pose2d pose2d = robot.getPose();
        if (pose2d == null) {
            Match.log("Could not get pose");
            return;
        }
        field.strokeCircle(pose2d.getX(), pose2d.getY(), .2);

        double rotation = pose2d.getHeading();
        double sin = Math.sin(rotation);
        double cos = Math.cos(rotation);

        //calculate the four points of the robot as if it was sitting on the origin
        double x1 = (MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2*cos + MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2*sin)/Field.MM_PER_INCH;
        double y1 = (MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2*sin - MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2*cos)/Field.MM_PER_INCH;
        double x2 = (MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2*cos - MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2*sin)/Field.MM_PER_INCH;
        double y2 = (MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2*sin + MecanumDriveTrain.DRIVE_TRAIN_WIDTH/2*cos)/Field.MM_PER_INCH;
        double x3 = -x1;
        double y3 = -y1;
        double x4 = -x2;
        double y4 = -y2;

        //add the robot's X coordinate to all X
        x1 += pose2d.getX();
        x2 += pose2d.getX();
        x3 += pose2d.getX();
        x4 += pose2d.getX();
        //add the robot's Y coordinate to all Y
        y1 += pose2d.getY();
        y2 += pose2d.getY();
        y3 += pose2d.getY();
        y4 += pose2d.getY();


        //the point in front of the robot to create the triangle to show direction
        double px = (pose2d.getX() + (MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 + 100)*cos/Field.MM_PER_INCH);
        double py = (pose2d.getY() + (MecanumDriveTrain.DRIVE_TRAIN_LENGTH/2 + 100)*sin/Field.MM_PER_INCH);


        //draw our rectangular robot
        field.strokeLine(x1, y1, x2, y2);
        field.strokeLine(x2, y2, x3, y3);
        field.strokeLine(x3, y3, x4, y4);
        field.strokeLine(x4, y4, x1, y1);

        //draw two lines in the front to make the triangle to show direction
        field.strokeLine(x1, y1, px, py);
        field.strokeLine(x2, y2, px, py);

        packet.put("x", pose2d.getX());
        packet.put("y", pose2d.getY());
        packet.put("theta", pose2d.getHeading());

        dashboard.sendTelemetryPacket(packet);
    }
}
