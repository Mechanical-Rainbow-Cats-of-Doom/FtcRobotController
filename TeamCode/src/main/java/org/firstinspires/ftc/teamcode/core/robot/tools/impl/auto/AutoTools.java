package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.Timer;
import java.util.TimerTask;

import androidx.annotation.NonNull;

public class AutoTools {
    protected final DcMotor liftMotor, armMotor;
    protected final AutoToolRotation rotation;
    protected final CRServo intake;
    protected final Timer timer;
    public enum Action {
        INTAKE,
        DUMP,
        NOTHING
    }
    //armpos on dump is above
    public enum Position { // THESE VALUES ARE JUST GUESSES
        NEUTRAL(0, 80, Action.NOTHING),
        INTAKE(0, 25, Action.INTAKE),
        GROUND_TARGET(INTAKE.liftPos, 280, Action.DUMP),
        LOW_TARGET(0, 752, Action.DUMP),
        MEDIUM_TARGET(1251, 671, Action.DUMP),
        MAX(2523, 1000, Action.NOTHING), //armpos max is verified
        HIGH_TARGET(MAX.liftPos, 603, Action.DUMP),
        GROUND_TARGET_NODUMP(GROUND_TARGET.liftPos, GROUND_TARGET.armPos, Action.NOTHING),
        LOW_TARGET_NODUMP(LOW_TARGET.liftPos, LOW_TARGET.armPos, Action.NOTHING),
        MEDIUM_TARGET_NODUMP(MEDIUM_TARGET.liftPos, MEDIUM_TARGET.armPos, Action.NOTHING),
        HIGH_TARGET_NODUMP(HIGH_TARGET.liftPos, HIGH_TARGET.armPos, Action.NOTHING);

        public final int liftPos;
        public final int armPos;
        final Action action;
        Position(int liftPos, int armPos, Action action) {
            this.liftPos = liftPos;
            this.armPos = armPos;
            this.action = action;
        }
    }

    private Position position = Position.NEUTRAL;
    private Position lastPosition = Position.MAX;
    /*
     * The stage represents how far the robot is from getting to the correct position.
     * 0 - initial position, hasn't started moving
     * 1 - started moving, hasn't reached destination
     * 2 - finished moving, hasn't optionally dumped
     * 3 - finished.
     * If the position doesn't dump, stage 2 is skipped.
     */
    protected int stage = 0;
    protected boolean waiting = true;
    protected boolean doingstuff = false;
    protected boolean isAuto = true;
    public AutoTools(@NonNull HardwareMap hardwareMap, Timer timer, AutoToolRotation rotation) {
        this.liftMotor = hardwareMap.get(DcMotor.class, "lift");
        this.armMotor = hardwareMap.get(DcMotor.class, "arm");
        this.intake = hardwareMap.get(CRServo.class, "intake");
        this.rotation = rotation;
        this.timer = timer;
        initMotors();
    }

    protected void initMotors() {
        ZeroMotorEncoder.zero(liftMotor);
        ZeroMotorEncoder.zero(armMotor);
    }

    public void setPosition(@NonNull Position position) {
        this.position = position;
    }

    protected void dump() {
        doingstuff = true;
        if (!isAuto) {
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        final int startPos = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(startPos-50);
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                intake.setPower(1);
                timer.schedule(new TimerTask() {
                    @Override
                    public void run() {
                        armMotor.setTargetPosition(startPos);
                        Thread thread = new Thread(() -> {
                            while (armMotor.isBusy()) {
                                try {
                                    //noinspection BusyWait
                                    Thread.sleep(60);
                                } catch (InterruptedException ignored) {}
                            }
                            if (isAuto) stage++;
                            else {
                                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            }
                            intake.setPower(0);
                            doingstuff = false;
                        });
                        thread.start();
                    }
                }, 300);
            }
        }, 60);
    }
    public void update() {
        if (isAuto) doingstuff = true;
        else {
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(lastPosition != position) {
            this.stage = 0;
            this.lastPosition = position;
            waiting = false;
        }
        switch (stage) {
            // initial position
            case 0:
                if(!waiting) {
                    liftMotor.setTargetPosition(position.liftPos);
                    armMotor.setTargetPosition(position.armPos);
                    if (position.action == Action.INTAKE) intake.setPower(-1);
                }
                stage++;
                break;
            case 1:
                 if(!liftMotor.isBusy() && !armMotor.isBusy()) {
                    stage++;
                    if (position.action == Action.INTAKE) intake.setPower(0);
                 }
                 break;
            case 2:
                if (!waiting) {
                    waiting = true;
                    if (position.action == Action.DUMP) {
                        dump();
                    } else if (position.action == Action.INTAKE) {
                        timer.schedule(new TimerTask() {
                            @Override
                            public void run() {
                                intake.setPower(0);
                                stage++;
                                waiting = false;
                            }
                        }, 350);
                    } else {
                        stage++;
                        waiting = false;
                    }
                }
                break;
            case 3:
                if (position == Position.NEUTRAL) {
                    waiting = true;
                } else if ((isAuto && position.action != Action.NOTHING) || position == Position.INTAKE) position = Position.NEUTRAL;
                if (!isAuto) {
                    doingstuff = false;
                    armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                stage = 0;
                break;
        }
    }
}
