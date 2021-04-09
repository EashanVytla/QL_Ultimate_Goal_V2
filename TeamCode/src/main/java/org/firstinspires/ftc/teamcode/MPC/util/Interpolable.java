package org.firstinspires.ftc.teamcode.MPC.util;

public interface Interpolable<T> {
    T interpolate(T other, double x);
}
