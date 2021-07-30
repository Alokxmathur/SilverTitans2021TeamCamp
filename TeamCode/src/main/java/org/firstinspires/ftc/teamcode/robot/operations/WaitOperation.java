package org.firstinspires.ftc.teamcode.robot.operations;

import java.util.Date;
import java.util.Locale;

/**
 * Created by Silver Titans on 10/12/17.
 */

public class WaitOperation extends Operation {
    public float getTime() {
        return time;
    }

    private long time;

    public WaitOperation(long time, String title) {
        this.type = TYPE.WAIT_TIME;
        this.time = time;
        this.title = title;
    }

    public String toString() {
        return String.format(Locale.getDefault(),"WaitOperation: for %d msecs --%s",
                this.time, this.title);
    }

    public boolean isComplete() {
        return new Date().getTime() - getStartTime().getTime() > time;
    }
}
