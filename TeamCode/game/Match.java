package org.firstinspires.ftc.teamcode.game;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;

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
    private Date startTime = new Date();
    private Date teleopStartTime = new Date();
    private int numberOfRings;
    private Alliance.Color allianceColor;

    public int getNumberOfRings() {
        return numberOfRings;
    }

    public void setNumberOfRings(int numberOfRings) {
        this.numberOfRings = numberOfRings;
    }

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

    /**
     * Give the driver station a state of the union
     *
     * @param telemetry
     */
    public void updateTelemetry(Telemetry telemetry, String status) {

        if (robot != null && field != null) {
            // Send telemetry message to signify robot context;
            telemetry.addData("State:", status);
            telemetry.addData("M", robot.getMotorStatus());
            //telemetry.addData("S", robot.getSensorStatus());

            telemetry.addData("Pos", robot.getPosition());
            telemetry.addData("Bearing", robot.getBearing());
            telemetry.addData("Picker", robot.getPickerArmStatus());
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
}
