package org.firstinspires.ftc.teamcode.game;

/**
 * Created by Silver Titans on 11/19/17.
 */

public class Coordinates {
    double x, y, z;

    public Coordinates(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Coordinates(Coordinates coordinates) {
        this.x = coordinates.getX();
        this.y = coordinates.getY();
        this.z = coordinates.getZ();
    }

    public double getX() {
        return x;
    }
    public double getXInInches() {
        return x/Field.MM_PER_INCH;
    }

    public void setX(float x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }
    public double getYInInches() {
        return y/Field.MM_PER_INCH;
    }

    public void setY(float y) {
        this.y = y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(float z) {
        this.z = z;
    }

    public void incrementX(double increment) {
        this.x += increment;
    }
    public void incrementY(double increment) {
        this.y += increment;
    }
    public void incrementZ(double increment) {
        this.z += increment;
    }
}
