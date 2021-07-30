package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.components.PickerArm;

import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class PickerOperation extends Operation {
    double winchExtension;
    int shoulderPosition;

    public enum PickerOperationType {
        INITIAL, SHOULDER_LEVEL, SHOULDER_RELEASE, SHOULDER_GRAB, SHOULDER_VERTICAL, SHOULDER_LIFT, OPEN_GRIPPER, CLOSE_GRIPPER,
        CARRY, HOVER, PLACE, ARM_EXTENSION, SHOULDER_POSITION, EXTENSION_AND_SHOULDER_POSITION,
        LEVEL_1, LEVEL_2, LEVEL_3, LEVEL_4, LEVEL_5, LEVEL_6, LEVEL_7, UP, OUT, GRAB
    }

    public PickerOperationType getPickerOperationType() {
        return pickerOperationType;
    }

    PickerOperationType pickerOperationType;

    public double getWinchExtension() {
        return winchExtension;
    }

    public void setWinchExtension(double winchExtension) {
        this.winchExtension = winchExtension;
    }

    public int getShoulderPosition() {
        return shoulderPosition;
    }

    public void setShoulderPosition(int shoulderPosition) {
        this.shoulderPosition = shoulderPosition;
    }

    public PickerOperation(PickerOperationType pickerOperationType, String title) {
        this.type = TYPE.PICKER_OPERATION;
        this.pickerOperationType = pickerOperationType;
        this.title = title;
    }

    public PickerOperation(double winchExtension, String title) {
        this.type = TYPE.PICKER_OPERATION;
        this.pickerOperationType = PickerOperationType.ARM_EXTENSION;
        this.setWinchExtension(winchExtension);
        this.title = title;
    }

    public PickerOperation(double winchExtension, int shoulderPosition, String title) {
        this.type = TYPE.PICKER_OPERATION;
        this.pickerOperationType = PickerOperationType.EXTENSION_AND_SHOULDER_POSITION;
        this.setWinchExtension(winchExtension);
        this.setShoulderPosition(shoulderPosition);
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(), "Picker: %s --%s",
                this.pickerOperationType,
                this.title);
    }


    public boolean isComplete(PickerArm picker) {
        return picker.isComplete(this);
    }
}

