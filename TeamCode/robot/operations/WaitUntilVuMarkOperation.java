package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class WaitUntilVuMarkOperation extends Operation {
    public float getTime() {
        return time;
    }

    private long time;
    private Date timeStarted;

    public WaitUntilVuMarkOperation(long time, String title) {
        this.type = TYPE.WAIT_UNTIL_VUMARK;
        this.time = time;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"WaitUntilVuMarkOperation: for %d msecs --%s",
                this.time, this.title);
    }

    public void setStart() {
        this.timeStarted = new Date();
    }
    public boolean isComplete(Robot robot) {
        return robot.findTarget() != null || (new Date().getTime() - timeStarted.getTime() > time);
    }
}
