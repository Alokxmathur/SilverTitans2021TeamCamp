package org.firstinspires.ftc.teamcode.robot.components.imu;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

public class IMU {
    private BNO055IMU imu;
    private double initialBearing;
    private double initialRoll;
    private double initialPitch;
    private double initialX;
    private double initialY;
    private double initialZ;

    private boolean calibratedWithKnownLocation = false;

    private Orientation angles;
    private Acceleration gravity;

    public IMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.useExternalCrystal = true;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.loggingTag = "Silver Titans IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
    }

    public void calibrateWithKnownLocation(OpenGLMatrix knownLocation) {
        //first get our state from the IMU
        update();
        //now add in what the known location is telling us
        Orientation rotation = Orientation.getOrientation(knownLocation, EXTRINSIC, XYZ, DEGREES);
        VectorF translation = knownLocation.getTranslation();
        this.initialPitch = rotation.firstAngle
                - angles.firstAngle;
        this.initialRoll = 
                rotation.secondAngle - angles.secondAngle;
        this.initialBearing = 
                rotation.thirdAngle - angles.thirdAngle;
        this.initialX = translation.get(0);
        this.initialY = translation.get(1);
        this.initialZ = translation.get(2);

        this.calibratedWithKnownLocation = true;
    }

    public boolean isCalibratedWithKnownLocation() {
        return calibratedWithKnownLocation;
    }

    private void update() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        gravity = imu.getGravity();
    }

    public double getPitch() {
        update();
        return angles.firstAngle + initialPitch;
    }

    public double getRoll() {
        update();
        return angles.secondAngle + initialRoll;
    }

    public double getBearing() {
        update();
        return AngleUnit.normalizeDegrees(angles.thirdAngle + initialBearing);
    }

    public double getRawBearing() {
        update();
        return AngleUnit.normalizeDegrees(angles.thirdAngle);
    }

    public Acceleration getAcceleration() {
        update();
        return gravity;
    }

    public String getStatus() {
        return String.format("{R, P, B} = %.0f(%.0f), %.0f(%.0f), %.0f(%.0f)",
                getRoll(), angles.secondAngle, getPitch(),
                angles.firstAngle, getBearing(), angles.thirdAngle);

    }

    public boolean isCalibrated() {
        return this.imu.isGyroCalibrated();
    }
}
