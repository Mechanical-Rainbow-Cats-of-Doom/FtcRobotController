package org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.robot.tools.BetterDistanceSensor;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.Cycler;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.driveop.ControllerTools;
import org.firstinspires.ftc.teamcode.core.robot.util.ZeroMotorEncoder;

import java.util.Timer;
import java.util.TimerTask;
import java.util.function.BooleanSupplier;

import androidx.annotation.NonNull;

@Config
public class AutoTools {
    private final LinearOpMode opMode;
    public static double armZeroPower = 0.075, liftZeroPower = 0.001;
    protected final DcMotor liftMotor, armMotor, cyclingMotor;
    private final BetterDistanceSensor distanceSensor;
    protected AutoTurret turret;
    protected final CRServo intake;
    protected final Timer timer;
    public enum Action {
        INTAKE,
        DUMP,
        NOTHING
    }

    //armpos on dump is above
    public enum Position {
        NEUTRAL(0, 80, Action.NOTHING),
        OFF(0, 0, Action.NOTHING),
        INTAKE(0, 25, Action.INTAKE),
        INTAKE_NO_INTAKE(INTAKE.liftPos, INTAKE.armPos, Action.NOTHING),
        GROUND_TARGET(INTAKE.liftPos, 280, Action.DUMP),
        LOW_TARGET(0, 752, Action.DUMP),
        MEDIUM_TARGET(1251, 671, Action.DUMP),
        MAX(2523, 1000, Action.NOTHING), //armpos max is verified
        HIGH_TARGET(1875, MAX.armPos, Action.DUMP),
        GROUND_TARGET_NODUMP(GROUND_TARGET.liftPos, GROUND_TARGET.armPos, Action.NOTHING),
        LOW_TARGET_NODUMP(LOW_TARGET.liftPos, LOW_TARGET.armPos, Action.NOTHING),
        MEDIUM_TARGET_NODUMP(MEDIUM_TARGET.liftPos, MEDIUM_TARGET.armPos, Action.NOTHING),
        HIGH_TARGET_NODUMP(HIGH_TARGET.liftPos, HIGH_TARGET.armPos, Action.NOTHING),
        HIGH_ARM(0, MAX.armPos, Action.NOTHING),
        //cone 5
        HOVER_5(900,0,Action.NOTHING),
        INTAKE_5(600,0,Action.NOTHING),
        EXIT_5(1250,0,Action.NOTHING),
        //cone 4
        HOVER_4(750,0,Action.NOTHING),
        INTAKE_4(550,0,Action.NOTHING),
        EXIT_4(1050,0,Action.NOTHING),
        //cone 3
        HOVER_3(570,0,Action.NOTHING),
        INTAKE_3(370,0,Action.NOTHING),
        EXIT_3(900,0,Action.NOTHING),
        //cone 2
        HOVER_2(360,0,Action.NOTHING),
        INTAKE_2(160,0,Action.NOTHING),
        EXIT_2(700,0,Action.NOTHING),
        //cone 1
        HOVER_1(200,0,Action.NOTHING),
        INTAKE_1(0,0,Action.NOTHING),
        EXIT_1(500,0,Action.NOTHING);

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
    protected boolean cycling = false;
    protected boolean needsToChangeMode = true;
    protected Cycler cycler;
    protected final ControllerTools.BoxedBoolean doingstuff = new ControllerTools.BoxedBoolean();
    protected boolean isAuto = true;
    private final Telemetry telemetry;
    public AutoTools(@NonNull HardwareMap hardwareMap, Timer timer, AutoTurret turret, LinearOpMode opMode, Telemetry telemetry) {
        this.distanceSensor = new BetterDistanceSensor(hardwareMap, "distanceSensor", 50, DistanceUnit.CM);
        this.liftMotor = hardwareMap.get(DcMotor.class, "lift");
        this.armMotor = hardwareMap.get(DcMotor.class, "arm");
        this.cyclingMotor = hardwareMap.get(DcMotor.class, "cycle");
        this.intake = hardwareMap.get(CRServo.class, "intake");
        this.turret = turret;
        this.timer = timer;
        this.opMode = opMode;
        this.telemetry = telemetry;
        initMotors();
    }

    protected void initMotors() {
        ZeroMotorEncoder.zero(liftMotor, cyclingMotor);
        ZeroMotorEncoder.zero(armMotor, 1);
    }

    public void setPosition(@NonNull Position position) {
        this.stage = 0;
        this.position = position;
    }
    public boolean setCycler(Cycler.Cycles cycle, int howManyCones) {
        if (!cycling) cycler = new Cycler(distanceSensor, liftMotor, armMotor, cyclingMotor, turret, this::setIntake, cycle, this::stopCycling, howManyCones);
        return !cycling;
    }
    public boolean setCycler(Cycler.Cycles cycle, BooleanSupplier shouldEnd) {
        if (!cycling) cycler = new Cycler(distanceSensor, liftMotor, armMotor, cyclingMotor, turret, this::setIntake, cycle, this::stopCycling, shouldEnd);
        return !cycling;
    }
    public void startCycling(Cycler.Cycles cycle, int howManyCones) {
        if (setCycler(cycle, howManyCones)) startCycling();
    }
    public void startCycling(Cycler.Cycles cycle, BooleanSupplier shouldEnd) {
        if (setCycler(cycle, shouldEnd)) startCycling();
    }
    public void startCycling() {
        cycling = true;
        doingstuff.value = true;
    }
    public void stopCycling() {
        cycling = false;
        waiting = true;
        doingstuff.value = false;
        position = Position.NEUTRAL; // probably not needed
    }
    public void update() {
        if (!isAuto && needsToChangeMode) {
            try {
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                needsToChangeMode = false;
            } catch (TargetPositionNotSetException ignored) {
                // who
            }
        }
        if (!cycling) {
            if (lastPosition != position) {
                this.stage = 0;
                this.lastPosition = position;
                waiting = false;
            }
            switch (stage) {
                // initial position
                case 0:
                    if (!waiting) {
                        doingstuff.value = true;
                        liftMotor.setTargetPosition(position.liftPos);
                        armMotor.setTargetPosition(position.armPos);
                        liftMotor.setPower(1);
                        armMotor.setPower(1);
                        if (position.action == Action.INTAKE) intake.setPower(-1);
                    }
                    stage++;
                    break;
                case 1:
                    if (!liftMotor.isBusy() && !armMotor.isBusy()) {
                        stage++;
                        if (position.action == Action.INTAKE) intake.setPower(0);
                    }
                    break;
                case 2:
                    if (!waiting) {
                        waiting = true;
                        if (position.action == Action.INTAKE) {
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
                    } else if ((isAuto && position.action != Action.NOTHING) || position == Position.INTAKE)
                        position = Position.NEUTRAL;
                    if (!isAuto) {
                        doingstuff.value = false;
                        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        needsToChangeMode = true;
                        armMotor.setPower(armZeroPower);
                        liftMotor.setPower(liftZeroPower);
                    }
                    stage = 0;
                    if (isAuto) {
                        doingstuff.value = false;
                    }
                    break;
            }
        } else {
            telemetry.addData("cones cycled", cycler.getConesDumped());
            cycler.update();
        }
    }

    public boolean isDoingStuff() {
        return doingstuff.value;
    }

    public void waitUntilFinished() {
        //noinspection StatementWithEmptyBody
        while(doingstuff.value && !opMode.isStopRequested());
    }

    public void waitUntilFinished(BooleanSupplier booleanSupplier) {
        //noinspection StatementWithEmptyBody
        while(doingstuff.value && !opMode.isStopRequested() && booleanSupplier.getAsBoolean());
    }

    public void setIntake(Action action) {
        intake.setPower(action == Action.INTAKE ? 1 : action == Action.DUMP ? -1 : 0);
    }

    public void setIntake(int power){
        intake.setPower(power);
    }

    public void cleanup() {
        armMotor.setPower(armZeroPower);
        liftMotor.setPower(liftZeroPower);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        needsToChangeMode = true;
        turret.cleanup();
    }

    public double getTurretPos(AutoTurret.Units unit) {
        return turret.getPos(unit);
    }
}
