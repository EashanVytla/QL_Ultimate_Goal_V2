package org.firstinspires.ftc.teamcode.Wrapper;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadEx {
    public Gamepad gamepad;

    private boolean a_click;
    private boolean x_click;
    private boolean y_click;
    private boolean dpad_down_click;
    private boolean left_stick_button_click;
    private boolean right_bumper_click;
    private boolean left_bumper_click;
    private boolean b_click;
    private boolean back_click;
    private boolean dpad_left_click;
    private boolean dpad_right_click;
    private boolean dpad_up_click;
    private boolean right_stick_button_click;
    private boolean start_click;
    private boolean right_trigger_click;
    private boolean left_trigger_click;
    private boolean touchpad_click;

    private boolean a_prev;
    private boolean x_prev;
    private boolean y_prev;
    private boolean dpad_down_prev;
    private boolean left_stick_button_prev;
    private boolean right_bumper_prev;
    private boolean left_bumper_prev;
    private boolean b_prev;
    private boolean back_prev;
    private boolean dpad_left_prev;
    private boolean dpad_right_prev;
    private boolean dpad_up_prev;
    private boolean right_stick_button_prev;
    private boolean start_prev;
    private boolean right_trigger_prev;
    private boolean left_trigger_prev;
    private boolean touchpad_prev;

    public enum Control{
        a,
        x,
        y,
        dpad_down,
        dpad_right,
        left_stick_button,
        right_bumper,
        left_bumper,
        left_trigger,
        right_trigger,
        b,
        back,
        dpad_left,
        dpad_up,
        right_stick_button,
        start,
        touchpad
    }

    public GamepadEx(Gamepad gamepad){
        this.gamepad = gamepad;
    }

    public boolean isPress(Control control){
        switch (control){
            case a:
                return a_click;
            case b:
                return b_click;
            case x:
                return x_click;
            case y:
                return y_click;
            case dpad_down:
                return dpad_down_click;
            case left_stick_button:
                return left_stick_button_click;
            case right_bumper:
                return right_bumper_click;
            case back:
                return back_click;
            case dpad_left:
                return dpad_left_click;
            case dpad_right:
                return dpad_right_click;
            case dpad_up:
                return dpad_up_click;
            case right_stick_button:
                return right_stick_button_click;
            case left_bumper:
                return left_bumper_click;
            case start:
                return start_click;
            case left_trigger:
                return left_trigger_click;
            case right_trigger:
                return right_trigger_click;
            case touchpad:
                return touchpad_click;
            default:
                return false;
        }
    }

    public void loop(){
        //LOGITECH CONTROLLER
        //----------------------------------------------------------------------------------------//
        /*boolean a = gamepad.a;
        boolean x = gamepad.x;
        boolean y = gamepad.y;
        boolean dpad_down = gamepad.dpad_down;
        boolean left_stick_button = gamepad.left_stick_button;
        boolean right_bumper = gamepad.right_bumper;
        boolean left_bumper = gamepad.left_bumper;
        boolean b = gamepad.b;
        boolean back = gamepad.back;
        boolean dpad_left = gamepad.dpad_left;
        boolean dpad_right = gamepad.dpad_left;
        boolean dpad_up = gamepad.dpad_up;
        boolean right_stick_button = gamepad.right_stick_button;
        boolean start = gamepad.start;
        boolean right_trigger = gamepad.right_trigger > 0.3;
        boolean left_trigger = gamepad.left_trigger > 0.3;*/
        //----------------------------------------------------------------------------------------//

        //PS4 CONTROLLER
        //----------------------------------------------------------------------------------------//
        boolean a = gamepad.cross;
        boolean x = gamepad.square;
        boolean y = gamepad.triangle;
        boolean dpad_down = gamepad.dpad_down;
        boolean left_stick_button = gamepad.left_stick_button;
        boolean right_bumper = gamepad.right_bumper;
        boolean left_bumper = gamepad.left_bumper;
        boolean b = gamepad.b;
        boolean back = gamepad.back;
        boolean dpad_left = gamepad.dpad_left;
        boolean dpad_right = gamepad.dpad_right;
        boolean dpad_up = gamepad.dpad_up;
        boolean right_stick_button = gamepad.right_stick_button;
        boolean start = gamepad.options;
        boolean right_trigger = gamepad.right_trigger > 0.3;
        boolean left_trigger = gamepad.left_trigger > 0.3;
        boolean touchpad = gamepad.touchpad;
        //----------------------------------------------------------------------------------------//

        a_click = a && !a_prev;
        x_click = x && !x_prev;
        y_click = y && !y_prev;
        dpad_down_click = dpad_down && !dpad_down_prev;
        left_stick_button_click = left_stick_button && !left_stick_button_prev;
        right_bumper_click = right_bumper && !right_bumper_prev;
        left_bumper_click = left_bumper && !left_bumper_prev;
        b_click = b && !b_prev;
        back_click = back && !back_prev;
        dpad_left_click = dpad_left && !dpad_left_prev;
        dpad_right_click = dpad_right && !dpad_right_prev;
        dpad_up_click = dpad_up && !dpad_up_prev;
        right_stick_button_click = right_stick_button && !right_stick_button_prev;
        start_click = start && !start_prev;
        right_trigger_click = right_trigger && !right_trigger_prev;
        left_trigger_click = left_trigger && !left_trigger_prev;
        touchpad_click = touchpad && !touchpad_prev;

        a_prev = a;
        x_prev = x;
        y_prev = y;
        dpad_down_prev = dpad_down;
        left_stick_button_prev = left_stick_button;
        right_bumper_prev = right_bumper;
        left_bumper_prev = left_bumper;
        b_prev = b;
        back_prev = back;
        dpad_left_prev = dpad_left;
        dpad_right_prev = dpad_right;
        dpad_up_prev = dpad_up;
        right_stick_button_prev = right_stick_button;
        start_prev = start;
        right_trigger_prev = right_trigger;
        left_trigger_prev = left_trigger;
        touchpad_prev = gamepad.touchpad;
    }
}
