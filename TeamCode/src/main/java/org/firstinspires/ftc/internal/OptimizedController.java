package org.firstinspires.ftc.internal;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;

public class OptimizedController {
    private final Gamepad internalGamepad;

    public OptimizedController(Gamepad gamepad) {
        this.internalGamepad = gamepad;
    }
    public HashMap<Key, Boolean> beforeStateList = new HashMap<>();
    {
        beforeStateList.put(Key.START, false);
        beforeStateList.put(Key.A, false);
        beforeStateList.put(Key.B, false);
        beforeStateList.put(Key.X, false);
        beforeStateList.put(Key.Y, false);
        beforeStateList.put(Key.LEFT_BUMPER, false);
        beforeStateList.put(Key.RIGHT_BUMPER, false);
        beforeStateList.put(Key.LEFT_TRIGGER, false);
        beforeStateList.put(Key.RIGHT_TRIGGER, false);
        beforeStateList.put(Key.LEFT_STICK_X, false);
        beforeStateList.put(Key.LEFT_STICK_Y, false);
        beforeStateList.put(Key.RIGHT_STICK_X, false);
        beforeStateList.put(Key.RIGHT_STICK_Y, false);
        beforeStateList.put(Key.DPAD_UP, false);
        beforeStateList.put(Key.DPAD_DOWN, false);
        beforeStateList.put(Key.DPAD_LEFT, false);
        beforeStateList.put(Key.DPAD_RIGHT, false);
        beforeStateList.put(Key.BACK, false);
    }
    public enum Key {
        START, A, B, X, Y, LEFT_BUMPER, RIGHT_BUMPER, LEFT_TRIGGER, RIGHT_TRIGGER, LEFT_STICK_X, LEFT_STICK_Y, RIGHT_STICK_X, RIGHT_STICK_Y, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT, BACK
    }
    public boolean getBool(Key key) {
        if(key == Key.A){
            return internalGamepad.a;
        } else if(key == Key.B){
            return internalGamepad.b;
        } else if(key == Key.X){
            return internalGamepad.x;
        } else if(key == Key.Y){
            return internalGamepad.y;
        } else if(key == Key.LEFT_BUMPER){
            return internalGamepad.left_bumper;
        } else if(key == Key.RIGHT_BUMPER){
            return internalGamepad.right_bumper;
        } else if(key == Key.DPAD_UP){
            return internalGamepad.dpad_up;
        } else if(key == Key.DPAD_DOWN){
            return internalGamepad.dpad_down;
        } else if(key == Key.DPAD_LEFT){
            return internalGamepad.dpad_left;
        } else if(key == Key.DPAD_RIGHT){
            return internalGamepad.dpad_right;
        } else if(key == Key.START){
            return internalGamepad.start;
        } else if(key == Key.BACK){
            return internalGamepad.back;
        }
        return false;
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    public boolean getOnPress(Key key) {
        if(internalGamepad == null)
            return false;

        for(Key k : beforeStateList.keySet()) {
            if(k == key) {
                if(!beforeStateList.get(key) && getBool(key)) {
                    return !beforeStateList.replace(key, true);
                } else {
                    beforeStateList.replace(key, getBool(key));
                    return false;
                }
            }
        }
        return false;
    }
}
