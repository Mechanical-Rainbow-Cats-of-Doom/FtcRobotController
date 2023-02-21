package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.core.robot.tools.BetterDistanceSensor;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import androidx.annotation.NonNull;
@Config
public class Cycler {
    public static double isObjectDistance = 0;
    public static double dumpWaitTimeMs = 250, intakeWaitTimeMs = 150;
    private static int conesDumped = 0;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final BetterDistanceSensor distanceSensor;
    private final DcMotor liftMotor, armMotor, cyclingMotor;
    private final BooleanSupplier shouldEnd;
    private final AutoTurret turret;
    private final Consumer<AutoTools.Action> setIntake;
    private final Cycles cycle;
    private final Runnable stop;
    public Cycler(HardwareMap hardwareMap, DcMotor liftMotor, DcMotor armMotor, DcMotor cyclingMotor, AutoTurret turret, Consumer<AutoTools.Action> setIntake, Cycles cycle, Runnable stop, BooleanSupplier shouldEnd) {
        this.distanceSensor = new BetterDistanceSensor(hardwareMap, "distanceSensor", 50, DistanceUnit.CM);
        this.liftMotor = liftMotor;
        this.armMotor = armMotor;
        this.cyclingMotor = cyclingMotor;
        this.turret = turret;
        this.setIntake = setIntake;
        this.cycle = cycle;
        this.shouldEnd = shouldEnd;
        this.stop = stop;
        distanceSensor.start();
        distanceSensor.request(); //idk why this is there but kooky has it
    }
    public Cycler(HardwareMap hardwareMap, DcMotor liftMotor, DcMotor armMotor, DcMotor cyclingMotor, AutoTurret turret, Consumer<AutoTools.Action> setIntake, Cycles cycle, Runnable stopPipeline, int howManyCones) {
        this(hardwareMap, liftMotor, armMotor, cyclingMotor, turret, setIntake, cycle, stopPipeline, () -> conesDumped >= howManyCones);
    }
    public enum Steps {
        GO_TO_INTAKING,
        INTAKING,
        GO_TO_DUMP,
        DUMP
    }
    
    public enum Cycles {
        ALLIANCE_LEFT_HIGH(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0)
        ),
        ALLIANCE_RIGHT_HIGH(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0)
        ),
        STACK_LEFT_HIGH(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0)
        ),
        STACK_RIGHT_HIGH(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0)
        ),
        STACK_LEFT_MID(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0)
        ),
        STACK_RIGHT_MID(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0)
        );
        public final State intaking;
        public final State dumping;

        Cycles(State intaking, State dumping) {
            this.intaking = intaking;
            this.dumping = dumping;
        }
    }
    
    public static int getConesDumped() {
        return conesDumped;
    }
    private Steps step = Steps.GO_TO_INTAKING;
    private boolean ran = false, good = false;
    public static class State {
        public final int liftPos;
        public final int armPos;
        public final int cyclingPos;
        public final double turretPos;

        public State(int liftPos, int armPos, int cyclingPos, double turretPos) {
            this.liftPos = liftPos;
            this.armPos = armPos;
            this.cyclingPos = cyclingPos;
            this.turretPos = turretPos;
        }
    }
    private boolean ready() {
        return !(liftMotor.isBusy() || armMotor.isBusy() || cyclingMotor.isBusy() || turret.isMoving());
    }
    private void setToState(@NonNull State state) {
        liftMotor.setTargetPosition(state.liftPos);
        armMotor.setTargetPosition(state.armPos);
        cyclingMotor.setTargetPosition(state.cyclingPos);
        turret.setPos(state.turretPos, AutoTurret.Units.DEGREES);
    }
    public void update() {
        switch (step) {
            default:
            case GO_TO_INTAKING:
                if (!ran) {
                    setToState(cycle.intaking);
                    ran = true;
                } else if (ready()) {
                    step = Steps.INTAKING;
                    ran = false;
                }
                break;
            case INTAKING:
                if (!ran) {
                    setIntake.accept(AutoTools.Action.INTAKE);
                    armMotor.setTargetPosition(Math.max(cycle.intaking.armPos - 100, 0));
                }
                else if (good && timer.time(TimeUnit.MILLISECONDS) > intakeWaitTimeMs) {
                    step = Steps.GO_TO_DUMP;
                }
                else if (distanceSensor.request() < isObjectDistance) {
                    ran = false;
                    good = true;
                    timer.reset();
                }
                break;
            case GO_TO_DUMP:
                if (!ran) {
                    good = false;
                    setToState(cycle.dumping);
                    ran = true;
                } else if (ready()) {
                    step = Steps.DUMP;
                    ran = false;
                }
                break;
            case DUMP:
                if (!ran) {
                    armMotor.setTargetPosition(cycle.dumping.armPos - 100);
                    setIntake.accept(AutoTools.Action.DUMP);
                    ran = true;
                    timer.reset();
                } else if (timer.time() > dumpWaitTimeMs / 1.5) {
                    armMotor.setTargetPosition(cycle.dumping.armPos);
                } else if (timer.time() > dumpWaitTimeMs) {
                    step = Steps.INTAKING;
                    ++conesDumped;
                    ran = false;
                    if (shouldEnd.getAsBoolean()) {
                        cyclingMotor.setTargetPosition(0);
                        stop.run();
                    }
                }
                break;
        }
    }
}
