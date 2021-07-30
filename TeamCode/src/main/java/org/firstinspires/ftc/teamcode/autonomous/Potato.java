package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.operations.DriveForTimeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.IntakeOperation;
import org.firstinspires.ftc.teamcode.robot.operations.PickerOperation;
import org.firstinspires.ftc.teamcode.robot.operations.ShootingOperation;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Potato", group="Phoebe")

public class Potato extends OpMode {
    protected Match match;
    protected Robot robot;
    protected Field field;

    boolean initialOperationsQueued = false;

    @Override
    public void init() {
        this.match = Match.getNewInstance();
        Match.log("Created new match, initializing it");
        match.init();
        Match.log("Match initialized, setting alliance to " + Alliance.Color.RED + " and starting position to " + Field.StartingPosition.RIGHT);
        match.setAlliance(Alliance.Color.RED);
        match.setStartingPosition(Field.StartingPosition.RIGHT);
        field = match.getField();
        field.init();

        this.robot = match.getRobot();
        Match.log("Initializing robot");
        this.robot.init(hardwareMap, telemetry, match);
        Match.log("Robot initialized");

        robot.setPose(Field.STARTING_POSE);
        //robot.startVSLAM();

        robot.queuePrimaryOperation(new PickerOperation(PickerOperation.PickerOperationType.INITIAL, "Lift wobble goal slightly"));
        robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.TURN_ON_HIGH_GOAL, "Turn on flywheel for high goal"));

        telemetry.addData("Statuss", "Initialized, let's go");
        telemetry.update();

    }

    @Override
    public void init_loop() {
        //telemetry.addData("Status", "In init loop");
        //telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "I am running");
        telemetry.update();
        if (!initialOperationsQueued) {
            robot.queuePrimaryOperation(new DriveForTimeOperation(1000, 0, .4, "Move forward"));
            robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT, "Shoot"));

            robot.queuePrimaryOperation(new DriveForTimeOperation(1000, 0, .4, "Move forward"));
            robot.queuePrimaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOT, "Shoot"));

            robot.queueSecondaryOperation(new PickerOperation(PickerOperation.PickerOperationType.OUT, "Out with the wobble"));

            robot.queueTertiaryOperation(new ShootingOperation(ShootingOperation.ShootingOperationType.SHOOTING_POSITION,"Assume shooting position"));
            robot.queueTertiaryOperation(new IntakeOperation(IntakeOperation.IntakeOperationType.LIFT_PLATFORM, "Lift platform"));
            initialOperationsQueued = true;
        }
    }

    @Override
    public void stop() {
        telemetry.addData("Status", "I am stopped");
        telemetry.update();
    }
}
