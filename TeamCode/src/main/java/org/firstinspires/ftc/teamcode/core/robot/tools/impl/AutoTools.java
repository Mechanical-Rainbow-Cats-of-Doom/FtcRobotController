package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.Timer;
import java.util.TimerTask;

import androidx.annotation.NonNull;

public class AutoTools {
    final DcMotor liftMotor, armMotor;
    final AutoTurret turret;
    final CRServo intake;
    final Timer timer = new Timer();
    public enum Action {
        INTAKE,
        DUMP,
        NOTHING
    }
    //armpos on dump is above
    public enum Position { // THESE VALUES ARE JUST GUESSES
        NEUTRAL(0, 80, Action.NOTHING),
        INTAKE(0, 25, Action.INTAKE),
        GROUND_TARGET(INTAKE.liftPos, 25, Action.DUMP),
        LOW_TARGET(100, 300, Action.DUMP),
        MEDIUM_TARGET(200, 500, Action.DUMP),
        HIGH_TARGET(300,800, Action.DUMP),
        MAX(10000, 1000, Action.NOTHING), //armpos max is verified
        GROUND_TARGET_NODUMP(GROUND_TARGET.liftPos, GROUND_TARGET.armPos, Action.NOTHING),
        LOW_TARGET_NODUMP(LOW_TARGET.liftPos, LOW_TARGET.armPos, Action.NOTHING),
        MEDIUM_TARGET_NODUMP(MEDIUM_TARGET.liftPos, MEDIUM_TARGET.armPos, Action.NOTHING),
        HIGH_TARGET_NODUMP(HIGH_TARGET.liftPos, HIGH_TARGET.armPos, Action.NOTHING);

        final int liftPos;
        final int armPos;
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
    int stage = 0;
    boolean waiting = true;
    boolean doingstuff = false;
    boolean isAuto = true;
    public AutoTools(@NonNull HardwareMap hardwareMap, AutoTurret turret) {
        this.liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.armMotor = hardwareMap.get(DcMotor.class, "arm");
        this.intake = hardwareMap.get(CRServo.class, "intake");
        this.turret = turret;
        initMotors();
    }

    void initMotors() {
        ZeroMotorEncoder.zero(liftMotor);
        ZeroMotorEncoder.zero(armMotor);
    }

    public void setPosition(@NonNull Position position) {
        this.position = position;
    }

    void dump() {
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
                            if (!armMotor.isBusy()) {
                                //rotate turret so you don't smack yourself then
                                if (isAuto) stage++;
                                else {
                                    armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                    liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                                }
                                intake.setPower(0);
                                doingstuff = false;
                            } else try {
                                Thread.sleep(60);
                            } catch (InterruptedException ignored) {}
                        });
                        thread.start();
                    }
                }, 300);
            }
        }, 60);
    }
    public void update() {
        if (isAuto) doingstuff = true;
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
                if (position == Position.NEUTRAL) waiting = true;
                else if (isAuto || position == Position.INTAKE) position = Position.NEUTRAL;
                if (!isAuto) {
                    doingstuff = false;
                }
                stage = 0;
                break;
        }
    }
}
