package com.qualcomm.ftcrobotcontroller.opmodes.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Ethan Schaffer on 9/22/2016.
 */
public class contServo extends OpMode {
    Servo s;
    @Override
    public void init() {
        s = hardwareMap.servo.get("s");
    }

    @Override
    public void loop() {
        if(Math.abs(gamepad1.left_stick_y) > .1){
            s.setPosition(gamepad1.left_stick_y);
        } else {
            s.setPosition(0);
        }
    }
}
