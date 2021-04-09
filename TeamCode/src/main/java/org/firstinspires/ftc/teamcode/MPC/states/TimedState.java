package org.firstinspires.ftc.teamcode.MPC.states;

import org.firstinspires.ftc.teamcode.MPC.util.Time;
import org.firstinspires.ftc.teamcode.MPC.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.MPC.util.TimeUnits;

public abstract class TimedState<E extends Enum<E>> implements StateMachine<E> {
    private TimeProfiler timeProfiler;
    private E state;
    private E desiredState;

    protected TimedState(E state) {
        setTimeProfiler(new TimeProfiler(false));
        setState(state);
        setDesiredState(state);
    }

    protected abstract Time getStateTransitionDuration();

    @Override
    public void updateState(E state) {
        setDesiredState(state);
    }

    @Override
    public boolean hasReachedStateGoal() {
        return getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) >= getStateTransitionDuration().getTimeValue(TimeUnits.SECONDS);
    }

    @Override
    public boolean hasReachedStateGoal(E state) {
        return state.equals(getState()) && hasReachedStateGoal();
    }

    @Override
    public boolean attemptingStateChange() {
        return !getState().equals(getDesiredState());
    }

    @Override
    public E getState() {
        return state;
    }

    @Override
    public E getDesiredState() {
        return desiredState;
    }

    @Override
    public void update(double dt) {
        if(attemptingStateChange()) {
            getTimeProfiler().update(true);
            setState(getDesiredState());
        }
    }

    protected void setState(E state) {
        this.state = state;
    }

    private void setDesiredState(E state) {
        desiredState = state;
    }

    protected TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    protected void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }
}