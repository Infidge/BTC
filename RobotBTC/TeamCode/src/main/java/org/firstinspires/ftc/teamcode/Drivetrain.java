package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx rearLeft;
    public DcMotorEx rearRight;

    public Drivetrain(){
    }

    public void initDrivetrain(HardwareMap hwMap){

        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        rearLeft = hwMap.get(DcMotorEx.class, "rearLeft");
        rearRight = hwMap.get(DcMotorEx.class, "rearRight");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        rearLeft.setPower(0.0);
        rearRight.setPower(0.0);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void mecanumDrive(Gamepad gamepad){

        double drive = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double strafe = gamepad.left_stick_x;
        double flspeed, frspeed, rlspeed, rrspeed;
        double brake = 1.0;

        flspeed = drive + turn + strafe;
        frspeed = drive - turn - strafe;
        rlspeed = drive + turn - strafe;
        rrspeed = drive - turn + strafe;

        double max = Math.abs(Math.max(flspeed, Math.max(frspeed, Math.max(rrspeed, rlspeed))));
        if (max > 1.0){
            flspeed/=max;
            frspeed/=max;
            rlspeed/=max;
            rrspeed/=max;
        }

        frontLeft.setPower(flspeed);
        frontRight.setPower(frspeed);
        rearLeft.setPower(rlspeed);
        rearRight.setPower(rrspeed);

    }

}
