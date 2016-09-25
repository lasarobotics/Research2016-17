package com.qualcomm.ftcrobotcontroller.opmodes.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceReader;

import java.util.Arrays;

import navX.ftc.AHRS;

/**
 * Created by Ethan Schaffer on 9/14/2016.
 */
public class wallAlign extends LinearOpMode {
    public int getDistAway(I2cDeviceReader reader){
        byte[] rangeReadings = reader.getReadBuffer();
        return rangeReadings[0] & 0xFF;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left1 = hardwareMap.dcMotor.get("l1"); // AL00XR4D.1
        DcMotor left2 = hardwareMap.dcMotor.get("l2"); // AL00UYRR.1
        DcMotor right1 = hardwareMap.dcMotor.get("r1");// AL00XR4D.2
        DcMotor right2 = hardwareMap.dcMotor.get("r2"); // AL00UYRR.2
        left1.setDirection(DcMotor.Direction.REVERSE);
        left2.setDirection(DcMotor.Direction.REVERSE);
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get("dim");
        AHRS navx = AHRS.getInstance(dim, 0, AHRS.DeviceDataType.kProcessedData, (byte) 50);
        navx.zeroYaw();

        I2cDevice range = hardwareMap.i2cDevice.get("r"); // dim.port2
        I2cDeviceReader rangeReader = new I2cDeviceReader(range, 0x28, 0x04, 2);
        telemetry.addData("Dist", getDistAway(rangeReader));

        left1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        waitForStart();

        int target = 500;
        left1.setPower(.3);
        left2.setPower(.3);
        right1.setPower(.3);
        right2.setPower(.3);
        telemetry.addData("l1", left1.getTargetPosition());
        while(left1.isBusy()){
            left1.setTargetPosition(target);
            left2.setTargetPosition(target);
            right1.setTargetPosition(target);
            right2.setTargetPosition(target);
            waitOneFullHardwareCycle();
        }
        telemetry.addData("Status", "Done With Loop");
        while(getDistAway(rangeReader) > 10){
            telemetry.addData("Dist", getDistAway(rangeReader));
            arcade(0, 1, 0, left1, right1, left2, right2);
        }
        arcade(0, 0, 0, left1, right1, left2, right2);

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
