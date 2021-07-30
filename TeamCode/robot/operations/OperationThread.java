package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Date;

/**
 * Created by alokmathur on 10/29/17.
 */

public class OperationThread extends Thread {
    private final Object threadLock = new Object();
    private final String title;
    //stack of operationsQueue to perform
    private ArrayList<Operation> operationsQueue = new ArrayList<Operation>();
    private Robot robot;

    public OperationThread(Robot robot, String title) {
        this.robot = robot;
        this.title = title + " Operation Thread";
        Match.log(title + " created");
    }

    public void run() {
        Match.log(title + " started");
        while (true) {
            synchronized (threadLock) {
                //if we have performed the operation successfully,
                // we need to remove our current operation
                if (this.operationsQueue.size() > 0) {
                    Operation operation = this.operationsQueue.get(0);
                    if (operation.getOperationIsBeingProcessed()) {
                        if (robot.operationCompleted(operation)) {
                            this.operationsQueue.remove(0);
                            Match.log(title + ": Completed operation: " + operation.toString()
                                    + " at " + Match.getInstance().getElapsed()
                                    + " in " + (new Date().getTime() - operation.getStartTime().getTime())
                                    + " mSecs");
                        }
                    }
                }
                if (this.operationsQueue.size() > 0) {
                    Operation operation = this.operationsQueue.get(0);
                    if (!operation.getOperationIsBeingProcessed()) {
                        Match.log(title + ": Starting operation: " + operation.toString());
                        operation.setOperationBeingProcessed();
                        robot.executeOperation(operation);
                    }
                }
            }
            Thread.yield();
        }
    }

    public void queueUpOperation(Operation operation) {
        synchronized (threadLock) {
            this.operationsQueue.add(operation);
        }
    }

    public void abort() {
        synchronized (threadLock) {
            //if we performing an operation - abort it
            if (this.operationsQueue.size() > 0) {
                Operation operation = this.operationsQueue.get(0);
                if (operation.getOperationIsBeingProcessed()) {
                    robot.abortOperation(operation) ;
                }
            }
            this.operationsQueue = new ArrayList<Operation>();
        }
    }

    public boolean hasEntries() {
        synchronized (threadLock) {
            return this.operationsQueue.size() > 0;
        }
    }
}
