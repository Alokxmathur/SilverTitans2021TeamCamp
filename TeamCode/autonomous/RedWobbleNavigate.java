package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.game.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Red WobbleNav", group="Phoebe")
//@Disabled
public class RedWobbleNavigate extends AutonomousHelper {

    @Override
    public void init() {
        super.init(Alliance.Color.RED);
    }
}