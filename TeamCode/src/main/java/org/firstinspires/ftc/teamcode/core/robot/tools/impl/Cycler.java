package org.firstinspires.ftc.teamcode.core.robot.tools.impl;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.robot.tools.BetterDistanceSensor;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTools;
import org.firstinspires.ftc.teamcode.core.robot.tools.impl.auto.AutoTurret;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import androidx.annotation.NonNull;
@Config
public class Cycler {
    public static double isObjectDistance = 0;
    public static double dumpWaitTimeMs = 250, intakeWaitTimeMs = 150;
    private static int conesDumped = 0;
    private final double switchPoint;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final BetterDistanceSensor distanceSensor;
    private final DcMotor liftMotor, armMotor, cyclingMotor;
    private final BooleanSupplier shouldEnd;
    private final AutoTurret turret;
    private final Consumer<AutoTools.Action> setIntake;
    private final Cycles cycle;
    private final Thread wrapUpThread;

    private Steps step = Steps.GO_TO_INTAKING;
    private boolean ran = false, detected = false, movedin = false, firstrun = true, shouldEndVal = false;

    public Cycler(@NonNull BetterDistanceSensor distanceSensor, DcMotor liftMotor, DcMotor armMotor,
                  DcMotor cyclingMotor, AutoTurret turret, Consumer<AutoTools.Action> setIntake,
                  @NonNull Cycles cycle, Runnable stop, BooleanSupplier shouldEnd) {
        this.distanceSensor = distanceSensor;
        this.liftMotor = liftMotor;
        this.armMotor = armMotor;
        this.cyclingMotor = cyclingMotor;
        this.turret = turret;
        this.setIntake = setIntake;
        this.cycle = cycle;
        this.shouldEnd = shouldEnd;
        this.wrapUpThread = new Thread(() -> {
            cyclingMotor.setTargetPosition(0);
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) {}
            turret.setPos(0, AutoTurret.Units.DEGREES);
            liftMotor.setTargetPosition(AutoTools.Position.NEUTRAL.liftPos);
            armMotor.setTargetPosition(AutoTools.Position.NEUTRAL.armPos);
            while(!ready()) {
                try {
                    //noinspection BusyWait
                    Thread.sleep(50);
                } catch (InterruptedException ignored) {}
            }
            stop.run();
        });
        switchPoint = Math.abs(cycle.intaking.turretPos - cycle.dumping.turretPos) / 2;
        distanceSensor.start();
        distanceSensor.request(); //idk why this is there but kooky has it
    }

    @Override
    protected void finalize() throws Throwable {
        distanceSensor.stop();
        super.finalize();
    }

    /**
     * @param howManyCones Better be 5 or less if you are drawing from the stack
     */
    public Cycler(BetterDistanceSensor distanceSensor, DcMotor liftMotor, DcMotor armMotor,
                  DcMotor cyclingMotor, AutoTurret turret, Consumer<AutoTools.Action> setIntake,
                  Cycles cycle, Runnable stopPipeline, int howManyCones) {
        this(distanceSensor, liftMotor, armMotor, cyclingMotor, turret, setIntake, cycle, stopPipeline, () -> conesDumped >= howManyCones);
    }
    public enum Steps {
        GO_TO_INTAKING,
        INTAKING,
        GO_TO_DUMP,
        DUMP,
        DONE
    }
    
    public enum Cycles {
        ALLIANCE_LEFT_HIGH(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0),
                false
        ),
        ALLIANCE_RIGHT_HIGH(
                new State(ALLIANCE_LEFT_HIGH.intaking.liftPos, ALLIANCE_LEFT_HIGH.intaking.armPos, ALLIANCE_LEFT_HIGH.intaking.cyclingPos, 0),
                new State(ALLIANCE_LEFT_HIGH.dumping.liftPos, ALLIANCE_LEFT_HIGH.dumping.armPos, ALLIANCE_LEFT_HIGH.dumping.cyclingPos, 0),
                false
        ),
        STACK_LEFT_HIGH(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0),
                true
        ),
        STACK_RIGHT_HIGH(
                new State(STACK_LEFT_HIGH.intaking.liftPos, STACK_LEFT_HIGH.intaking.armPos, STACK_LEFT_HIGH.intaking.cyclingPos, 0),
                new State(STACK_LEFT_HIGH.dumping.liftPos, STACK_LEFT_HIGH.dumping.armPos, STACK_LEFT_HIGH.dumping.cyclingPos, 0),
                true
        ),
        STACK_LEFT_MID(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0),
                true
        ),
        STACK_RIGHT_MID(
                new State(STACK_LEFT_MID.intaking.liftPos, STACK_LEFT_MID.intaking.armPos, STACK_LEFT_MID.intaking.cyclingPos, 0),
                new State(STACK_LEFT_MID.dumping.liftPos, STACK_LEFT_MID.dumping.armPos, STACK_LEFT_MID.dumping.cyclingPos, 0),
                true
        );
        public final State intaking;
        public final State dumping;
        public final boolean stack;
        Cycles(State intaking, State dumping, boolean stack) {
            this.intaking = intaking;
            this.dumping = dumping;
            this.stack = stack;
        }
    }
    
    @SuppressWarnings("unused") // bro kys android studio
    public static int getConesDumped() {
        return conesDumped;
    }
    public static class State implements Cloneable {
        public final int liftPos;
        public final int armPos;
        public int cyclingPos; // for things
        public final double turretPos;

        public State(int liftPos, int armPos, int cyclingPos, double turretPos) {
            this.liftPos = liftPos;
            this.armPos = armPos;
            this.cyclingPos = cyclingPos;
            this.turretPos = turretPos;
        }

        public State setCyclingPos(int cyclingPos) {
            this.cyclingPos = cyclingPos;
            return this;
        }

        @NonNull
        @Override
        public State clone() {
            return new State(liftPos, armPos, cyclingPos, turretPos);
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
        if (!shouldEndVal) shouldEndVal = shouldEnd.getAsBoolean();
        switch (step) {
            case GO_TO_INTAKING:
                if (!ran) {
                    setToState(firstrun ? cycle.intaking : cycle.intaking.clone().setCyclingPos(0));
                    ran = true;
                } else if (!firstrun && !movedin && Math.abs(turret.getPos(AutoTurret.Units.DEGREES) - cycle.intaking.turretPos) <= switchPoint) {
                    cyclingMotor.setTargetPosition(cycle.intaking.cyclingPos);
                    movedin = true;
                } else if (ready()) {
                    step = Steps.INTAKING;
                    ran = false;
                    movedin = false;
                    firstrun = false;
                }
                break;
            case INTAKING:
                if (!ran) {
                    setIntake.accept(AutoTools.Action.INTAKE);
                    if (!cycle.stack) {
                        armMotor.setTargetPosition(Math.max(cycle.intaking.armPos - 100, 0));
                    } else {
                        liftMotor.setPower(0.6);
                        liftMotor.setTargetPosition(Math.max(cycle.intaking.liftPos - 100 * (conesDumped + 1), 0));
                    }
                    ran = true;
                } else if (!detected) {
                    if (distanceSensor.request() < isObjectDistance) {
                        if (cycle.stack) liftMotor.setPower(1);
                        detected = true;
                        timer.reset();
                    }
                } else if (timer.time() > intakeWaitTimeMs) {
                    ran = false;
                    detected = false;
                    step = Steps.GO_TO_DUMP;
                }
                break;
            case GO_TO_DUMP:
                if (!ran) {
                    setToState(cycle.dumping.clone().setCyclingPos(0));
                    ran = true;
                } else if (!movedin && Math.abs(turret.getPos(AutoTurret.Units.DEGREES) - cycle.dumping.turretPos) <= switchPoint) {
                    cyclingMotor.setTargetPosition(cycle.dumping.cyclingPos);
                    movedin = true;
                } else if (ready()) {
                    step = Steps.DUMP;
                    ran = false;
                    movedin = false;
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
                    ++conesDumped;
                    ran = false;
                    if (shouldEndVal) {
                        wrapUpThread.start();
                        step = Steps.DONE;
                    } else {
                        step = Steps.INTAKING;
                    }
                }
                break;
        }
    }
}
