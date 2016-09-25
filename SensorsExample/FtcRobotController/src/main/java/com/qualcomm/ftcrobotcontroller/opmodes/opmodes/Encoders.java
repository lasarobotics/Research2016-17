package com.qualcomm.ftcrobotcontroller.opmodes.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Ethan Schaffer on 8/15/2016.
 */
public class Encoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left1 = hardwareMap.dcMotor.get("l1"); // AL00XPH.port1
        DcMotor right1 = hardwareMap.dcMotor.get("r1"); // AL00XPH.port2
        DcMotor left2 = hardwareMap.dcMotor.get("l2"); // AL00VEA7.port1
        DcMotor right2 = hardwareMap.dcMotor.get("r2"); // AL00VEA7.port2
        right1.setDirection(DcMotor.Direction.REVERSE);
        right2.setDirection(DcMotor.Direction.REVERSE);

        left1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        int target = 500;
        left1.setPower(.15);
        left2.setPower(.15);
        right1.setPower(.15);
        right2.setPower(.15);

        int leftStartPos = left1.getCurrentPosition(), left2StartPos = left2.getCurrentPosition(),
                rightStartPos = right1.getCurrentPosition(), right2StartPos = right2.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("pos", left1.getCurrentPosition());
            if(left1.isBusy()){
                telemetry.addData("state", "busy");
            } else {
                telemetry.addData("state", "not busy");
                sleep(500);
                target = -target;
                left1.setTargetPosition(target + leftStartPos);
                left2.setTargetPosition(target + left2StartPos);
                right1.setTargetPosition(target + rightStartPos);
                right2.setTargetPosition(target + right2StartPos);
            }
            waitOneFullHardwareCycle();
        }
    }
}
