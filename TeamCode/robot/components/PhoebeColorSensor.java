package org.firstinspires.ftc.teamcode.robot.components;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by Silver Titans on 10/26/17.
 */

public class PhoebeColorSensor {
    public static final String SENSOR = "phoebeColorSensor";
    //our color sensor
    ColorSensor phoebeColorSensor = null;

    public PhoebeColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        // Define our sensor
        phoebeColorSensor = hardwareMap.get(ColorSensor.class, SENSOR);

    }
    public double getBlue() {
        return this.phoebeColorSensor.blue();
    }
    public double getRed() {
        return this.phoebeColorSensor.red();
    }
    public double getGreen() {
        return this.phoebeColorSensor.green();
    }
}
