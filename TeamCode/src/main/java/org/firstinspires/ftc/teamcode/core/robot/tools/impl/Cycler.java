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
    public static double dumpWaitTimeMs = 250, intakeWaitTimeMs = 75;
    private final double switchPoint;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private final BetterDistanceSensor distanceSensor;
    private final DcMotor liftMotor, armMotor;
    private final CyclerArm cyclerArm;
    private BooleanSupplier shouldEnd;
    private final AutoTurret turret;
    private final Consumer<AutoTools.Action> setIntake;
    private final Cycles cycle;
    private final Thread wrapUpThread;
    private final Runnable unsafeStop;

    private Steps step;
    private boolean ran = false, detected = false, movedin = false, firstrun = true, shouldEndVal = false;
    private int conesDumped = 0;

    public Cycler(@NonNull BetterDistanceSensor distanceSensor, DcMotor liftMotor, DcMotor armMotor,
                  CyclerArm cyclerArm, AutoTurret turret, Consumer<AutoTools.Action> setIntake,
                  @NonNull Cycles cycle, Runnable stop, BooleanSupplier shouldEnd, boolean startWithDump) {
        this.distanceSensor = distanceSensor;
        this.liftMotor = liftMotor;
        this.armMotor = armMotor;
        this.cyclerArm = cyclerArm;
        this.turret = turret;
        this.setIntake = setIntake;
        this.cycle = cycle;
        this.shouldEnd = shouldEnd; // do not call during constructor
        this.wrapUpThread = new Thread(() -> {
            cyclerArm.setTargetPosition(0);
            try {
                Thread.sleep(100);
            } catch (InterruptedException ignored) {}
            turret.setPos(0, AutoTurret.Units.DEGREES);
            liftMotor.setTargetPosition(AutoTools.Position.NEUTRAL.liftPos);
            armMotor.setTargetPosition(AutoTools.Position.NEUTRAL.armPos);
            while (!ready()) {
                try {
                    //noinspection BusyWait
                    Thread.sleep(50);
                } catch (InterruptedException ignored) {}
            }
            stop.run();
        });
        this.unsafeStop = stop;
        step = startWithDump ? Steps.GO_TO_DUMP : Steps.GO_TO_INTAKING;
        switchPoint = Math.abs(cycle.intaking.turretPos - cycle.dumping.turretPos) / 2;
        distanceSensor.start();
        distanceSensor.request(); //idk why this is there but kooky has it
    }

    /**
     * @param howManyCones Better be 5 or less if you are drawing from the stack
     */
    public Cycler(BetterDistanceSensor distanceSensor, DcMotor liftMotor, DcMotor armMotor,
                  CyclerArm cyclerArm, AutoTurret turret, Consumer<AutoTools.Action> setIntake,
                  Cycles cycle, Runnable stopPipeline, int howManyCones, boolean startWithDump) {
        this(distanceSensor, liftMotor, armMotor, cyclerArm, turret, setIntake, cycle, stopPipeline, null, startWithDump);
        shouldEnd = () -> conesDumped >= howManyCones;
        assert !(cycle.stack && howManyCones > 5);
    }

    public static class State  {
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

        public State cloneWithCyclingPosChanged(int cyclingPos) {
            return new State(liftPos, armPos, cyclingPos, turretPos);
        }

        public State cloneWithTurretPosChanged(double turretPos) {
            return new State(liftPos, armPos, cyclingPos, turretPos);
        }
    }

    private enum Steps {
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
                ALLIANCE_LEFT_HIGH.intaking.cloneWithTurretPosChanged(0),
                ALLIANCE_LEFT_HIGH.dumping.cloneWithTurretPosChanged(0),
                false
        ),
        STACK_LEFT_HIGH(
                new State(0, 0, 0, 0),
                new State(0, 0, 0, 0),
                true
        ),
        STACK_RIGHT_HIGH(
                STACK_LEFT_HIGH.intaking.cloneWithTurretPosChanged(0),
                STACK_LEFT_HIGH.dumping.cloneWithTurretPosChanged(0),
                true
        ),
        STACK_LEFT_MID(
                STACK_LEFT_HIGH.intaking,
                new State(0, 0, 0, 0),
                true
        ),
        STACK_RIGHT_MID(
                STACK_RIGHT_HIGH.intaking,
                STACK_LEFT_MID.dumping.cloneWithTurretPosChanged(0),
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

    public int getConesDumped() {
        return conesDumped;
    }

    private boolean ready() {
        return !(liftMotor.isBusy() || armMotor.isBusy() || cyclerArm.isBusy() || turret.isMoving());
    }

    private void setToState(@NonNull State state) {
        liftMotor.setTargetPosition(state.liftPos);
        armMotor.setTargetPosition(state.armPos);
        cyclerArm.setTargetPosition(state.cyclingPos);
        turret.setPos(state.turretPos, AutoTurret.Units.DEGREES);
    }

    public void endEarly(boolean safe) {
        if (safe) this.shouldEndVal = true;
        else unsafeStop.run();
    }

    public void update() {
        if (!shouldEndVal) shouldEndVal = shouldEnd.getAsBoolean();
        switch (step) {
            case GO_TO_INTAKING:
                if (!ran) {
                    setToState(firstrun ? cycle.intaking : cycle.intaking.cloneWithCyclingPosChanged(0));
                    ran = true;
                } else if (!firstrun && !movedin && Math.abs(turret.getPos(AutoTurret.Units.DEGREES) - cycle.intaking.turretPos) <= switchPoint) {
                    cyclerArm.setTargetPosition(cycle.intaking.cyclingPos);
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
                        detected = true;
                        timer.reset();
                    }
                } else if (timer.time() > intakeWaitTimeMs && (!movedin || ready())) {
                    if (cycle.stack && !movedin) {
                        liftMotor.setTargetPosition(cycle.intaking.liftPos + 200);
                        liftMotor.setPower(1);
                        movedin = true;
                        break;
                    }
                    ran = false;
                    detected = false;
                    movedin = false;
                    step = Steps.GO_TO_DUMP;
                }
                break;
            case GO_TO_DUMP:
                if (!ran) {
                    setToState(cycle.dumping.cloneWithCyclingPosChanged(0));
                    ran = true;
                } else if (!movedin && Math.abs(turret.getPos(AutoTurret.Units.DEGREES) - cycle.dumping.turretPos) <= switchPoint) {
                    cyclerArm.setTargetPosition(cycle.dumping.cyclingPos);
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
                    } else step = Steps.INTAKING;
                }
                break;
        }
    }

    @Override
    protected void finalize() throws Throwable {
        distanceSensor.stop();
        super.finalize();
    }
}
