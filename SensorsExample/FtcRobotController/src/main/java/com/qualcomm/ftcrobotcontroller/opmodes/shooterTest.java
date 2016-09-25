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
        left.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            left.setPower(1);
            right.setPower(.95);
        } else if(gamepad1.b) {
            left.setPower(1);
            right.setPower(1);
        } else if(gamepad1.y) {
            left.setPower(.95);
            right.setPower(1);
        }else if(gamepad1.x){
            left.setPower(.9);
            right.setPower(1);
        } else {
            left.setPower(0);
            right.setPower(0);
        }
    }
}
