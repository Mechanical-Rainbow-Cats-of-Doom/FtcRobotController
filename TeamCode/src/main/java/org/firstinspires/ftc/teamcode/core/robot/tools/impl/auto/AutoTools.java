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
    protected final AutoTurret turret;
    protected final CRServo intake;
    protected final Timer timer = new Timer();
    public enum Action {
        INTAKE,
        DUMP,
        NOTHING
    }
    //armpos on dump is above
    public enum Position { // THESE VALUES ARE JUST GUESSES
        NEUTRAL(40, 80, Action.NOTHING),
        INTAKE(10, 25, Action.INTAKE),
        GROUND_TARGET(10, 25, Action.DUMP),
        LOW_TARGET(100, 300, Action.DUMP),
        MEDIUM_TARGET(200, 500, Action.DUMP),
        HIGH_TARGET(300,800, Action.DUMP),
        MAX(10000, 1000, Action.NOTHING); //armpos max is verified

        public final int liftPos;
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
    protected int stage = 0;
    protected boolean waiting = true;
    protected boolean dumping = false;

    public AutoTools(@NonNull HardwareMap hardwareMap, AutoTurret turret) {
        this.liftMotor = hardwareMap.get(DcMotor.class, "lift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.armMotor = hardwareMap.get(DcMotor.class, "arm");
        this.intake = hardwareMap.get(CRServo.class, "intake");
        this.turret = turret;
        initMotors();
    }

    protected void initMotors() {
        ZeroMotorEncoder.zero(liftMotor);
        ZeroMotorEncoder.zero(armMotor);
    }

    public void setPosition(@NonNull Position position) {
        this.position = position;
    }

    protected void dump(boolean incrementStage) {
        dumping = true;
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
                                if (incrementStage) stage++;
                                dumping = false;
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
                        dump(true);
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
                    stage = 0;
                }
                else position = Position.NEUTRAL;
                break;
        }
    }
}
