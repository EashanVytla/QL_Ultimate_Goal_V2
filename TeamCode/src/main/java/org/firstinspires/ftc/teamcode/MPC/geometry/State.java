package org.firstinspires.ftc.teamcode.MPC.geometry;

import org.firstinspires.ftc.teamcode.MPC.util.Interpolable;

public interface State<S> extends Interpolable<S> {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();
}
