package org.firstinspires.ftc.teamcode.core.thread.event.impl;

import com.arcrobotics.ftclib.gamepad.KeyReader;

import org.firstinspires.ftc.teamcode.core.thread.event.api.AbstractEvent;

public class ReaderUpdatedEvent extends AbstractEvent {
    private final Runnable runnable;
    private final KeyReader reader;
    private final long delayBetween;

    public ReaderUpdatedEvent(Runnable runnable, KeyReader reader) {
        this(runnable, reader, 10);
    }

    public ReaderUpdatedEvent(Runnable runnable, KeyReader reader, long delayBetween) {
        this.runnable = runnable;
        this.reader = reader;
        this.delayBetween = delayBetween;
    }

    @Override
    public void run() {
        runnable.run();
    }

    @Override
    public boolean shouldRun() {
        reader.readValue();
        return reader.stateJustChanged() && super.shouldRun();
    }

    @Override
    public long reschedule() {
        return System.currentTimeMillis() + delayBetween;
    }

    @Override
    public long schedule() {
        return System.currentTimeMillis() + delayBetween;
    }
}
