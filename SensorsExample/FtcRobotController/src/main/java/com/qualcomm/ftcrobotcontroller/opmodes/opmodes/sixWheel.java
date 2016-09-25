package com.qualcomm.ftcrobotcontroller.opmodes.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Ethan Schaffer on 9/6/2016.
 */
public class sixWheel extends OpMode {
    DcMotor left1, right1, left2, right2, left3, right3;

    @Override
    public void init() {
        left1  = hardwareMap.dcMotor.get("l1");
        left2  = hardwareMap.dcMotor.get("l2");
        left3  = hardwareMap.dcMotor.get("l3");
        right1 = hardwareMap.dcMotor.get("r1");
        right2 = hardwareMap.dcMotor.get("r2");
        right3 = hardwareMap.dcMotor.get("r3");
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);
        right3.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        int mod = 1;
        if(gamepad1.right_bumper){
            mod = 3;
        }
        if(Math.abs(gamepad1.left_stick_y) > .1 ){
            left1.setPower(gamepad1.left_stick_y / mod);
            left2.setPower(gamepad1.left_stick_y / mod);
            left3.setPower(gamepad1.left_stick_y / mod);
        } else {
            left1.setPower(0);
            left2.setPower(0);
            left3.setPower(0);
        }
        if(Math.abs(gamepad1.right_stick_y) > .1 ){
            right1.setPower(gamepad1.right_stick_y / mod);
            right2.setPower(gamepad1.right_stick_y / mod);
            right3.setPower(gamepad1.right_stick_y / mod);
        } else {
            right1.setPower(0);
            right2.setPower(0);
            right3.setPower(0);
        }
    }
}
