package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ethan Schaffer on 9/22/2016.
 */

public class shooterTest extends OpMode {
    DcMotor left, right;

    @Override
    public void init() {
        left= hardwareMap.dcMotor.get("l");
        right=hardwareMap.dcMotor.get("r");
        right.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            left.setPower(.9);
            right.setPower(.9);
        } else if(gamepad1.b) {
            left.setPower(.8);
            right.setPower(.8);
        } else if(gamepad1.y) {
            left.setPower(.7);
            right.setPower(.7);
        }else if(gamepad1.x){
            left.setPower(.6);
            right.setPower(.6);
        } else {
            left.setPower(0);
            right.setPower(0);
        }
    }
}
