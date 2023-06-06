package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpPIDLift", group="Robot")
//@Config
public class TeleOpPIDLift extends LinearOpMode {

    Drivetrain dt = new Drivetrain();
    Lift lift = new Lift();

    ElapsedTime liftReadTime =  new ElapsedTime();

    public static volatile PIDCoefficients pid = new PIDCoefficients(0, 0, 0);
    public static volatile int liftPosition = 0;

    public static final PIDCoefficients lastPid = new PIDCoefficients(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        dt.initDrivetrain(hardwareMap);
        lift.initLift(hardwareMap);

        waitForStart();
        liftReadTime.reset();

        while (opModeIsActive()) {
            if (liftReadTime.milliseconds() > 25) {
                lift.readLiftEncoders();
                liftReadTime.reset();
            }

            dt.mecanumDrive(gamepad1);

            lift.PIDControllerTuning(pid, liftPosition);

            if (lastPid.p != pid.p || lastPid.i != pid.i || lastPid.d != pid.d) lift.resetPIDError();
            lastPid.p = pid.p;
            lastPid.i = pid.i;
            lastPid.d = pid.d;
        }
    }

}
