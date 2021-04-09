package org.firstinspires.ftc.teamcode.MPC.states;

public interface StateMachine<E extends Enum<E>> {
    void updateState(E state);
    boolean hasReachedStateGoal();
    boolean hasReachedStateGoal(E state);
    boolean attemptingStateChange();
    void update(double dt);
    E getState();
    E getDesiredState();
}
