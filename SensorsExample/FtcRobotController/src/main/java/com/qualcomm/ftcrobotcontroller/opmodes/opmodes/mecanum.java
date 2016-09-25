package com.qualcomm.ftcrobotcontroller.opmodes.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

import navX.ftc.AHRS;

/**
 * Created by Ethan Schaffer on 9/12/2016.
 */
public class mecanum extends OpMode {
    DcMotor left1, left2, right1, right2;
    AHRS navX;
    @Override
    public void init() {
        left1 = hardwareMap.dcMotor.get("l1"); // AL00XR4D.1
        left2 = hardwareMap.dcMotor.get("l2"); // AL00UYRR.1
        right1 = hardwareMap.dcMotor.get("r1");// AL00XR4D.2
        right2 = hardwareMap.dcMotor.get("r2"); // AL00UYRR.2
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);
        /*
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get("dim");
        navX = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);
        navX.zeroYaw();
        */
    }

    @Override
    public void loop() {
        /*
        if(navX.getYaw()>0)
            fieldOriented(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_y, navX.getYaw(), left1, left2, right1, right2);
        */
        arcade(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, left1, right1, left2, right2);
    }
    public double normalize(double heading) {
        if (heading < 0) {
            return 360 - (Math.abs(heading) % 360);
        } else {
            return (heading % 360);
        }
    }
    public static void arcade(double y, double x, double c, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double leftFrontVal = y + x + c;
        double rightFrontVal = y - x - c;
        double leftBackVal = y - x + c;
        double rightBackVal = y + x - c;

        //Move range to between 0 and +1, if not already
        double[] wheelPowers = {rightFrontVal, leftFrontVal, leftBackVal, rightBackVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            leftFrontVal /= wheelPowers[3];
            rightFrontVal /= wheelPowers[3];
            leftBackVal /= wheelPowers[3];
            rightBackVal /= wheelPowers[3];
        }

        leftFront.setPower(leftFrontVal);
        rightFront.setPower(rightFrontVal);
        leftBack.setPower(leftBackVal);
        rightBack.setPower(rightBackVal);
    }

    /**
     * Implements the Arcade drive train with field orientation based on Gyro input
     *
     * @param y           The y-axis of the controller, forward/rev
     * @param x           The x-axis of the controller, strafe
     * @param c           The spin axis of the controller
     * @param gyroheading The current normalized gyro heading (between 0 and 360)
     * @param leftFront   The motor on the front left
     * @param rightFront  The motor on the front right
     * @param leftBack    The motor on the back left
     * @param rightBack   The motor on the back right
     */
    public void fieldOriented(double y, double x, double c, double gyroheading, DcMotor leftFront, DcMotor rightFront, DcMotor leftBack, DcMotor rightBack) {
        double cosA = Math.cos(Math.toRadians(normalize(gyroheading)));
        double sinA = Math.sin(Math.toRadians(normalize(gyroheading)));
        double xOut = x * cosA - y * sinA;
        double yOut = x * sinA + y * cosA;
        arcade(yOut, xOut, c, leftFront, rightFront, leftBack, rightBack);
    }
}
