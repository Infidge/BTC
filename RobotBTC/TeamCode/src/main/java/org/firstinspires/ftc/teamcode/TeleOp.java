package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Robot: Teleop POV", group="Robot")
public class TeleOp extends LinearOpMode {

    Drivetrain dt = new Drivetrain();
    Lift lift = new Lift();
    ElapsedTime sensorReadTime =  new ElapsedTime();
    ElapsedTime liftReadTime =  new ElapsedTime();
    ElapsedTime switchReadTime =  new ElapsedTime();
    ElapsedTime grabbedTime = new ElapsedTime();

    @Override
    public void runOpMode() {

        dt.initDrivetrain(hardwareMap);
        lift.initLift(hardwareMap);

        waitForStart();

        sensorReadTime.reset();
        liftReadTime.reset();
        switchReadTime.reset();

        while (opModeIsActive()) {

            /**Bulk Readings*/
            if (sensorReadTime.milliseconds() > 75) {
                lift.readSensors();
                sensorReadTime.reset();
            }

            if (liftReadTime.milliseconds() > 25){
                lift.readLiftEncoders();
                liftReadTime.reset();
            }

            if (switchReadTime.milliseconds() > 75) {
                lift.readSwitches();
                switchReadTime.reset();
            }

            /**Driving*/
            dt.mecanumDrive(gamepad1);

            /**Righting Stick*/
            if (gamepad1.a)
                lift.toggleUprightStickState();

            /**Lift and arm states*/
            if (gamepad2.dpad_up){
                lift.changeLiftState(Lift.LiftStates.HIGH);
                lift.changeGuideState(Lift.GuideStates.UP);
                lift.changeArmState(Lift.ArmStates.SCORE);
            }
            else if (gamepad2.dpad_left || gamepad2.dpad_right){
                lift.changeLiftState(Lift.LiftStates.MID);
                lift.changeGuideState(Lift.GuideStates.UP);
                lift.changeArmState(Lift.ArmStates.SCORE);
            }
            else if (gamepad2.dpad_down){
                lift.changeLiftState(Lift.LiftStates.LOW);
                lift.changeGuideState(Lift.GuideStates.UP);
                lift.changeArmState(Lift.ArmStates.SCORE);
            }
            else if (gamepad2.left_bumper){
                lift.changeLiftState(Lift.LiftStates.COLLECT);
                lift.changeGuideState(Lift.GuideStates.DOWN);
                lift.changeArmState(Lift.ArmStates.COLLECT);
            }

            /**Lift PID Controller*/
            lift.PIDController();

            /**Manual Toggles*/
            if (gamepad2.a)
                lift.toggleClawState();

            if (gamepad1.b)
                lift.toggleArmState();

            if (gamepad2.x)
                lift.toggleGuideState();

            /**Auto-Grab*/
            if (lift.checkForCone()) {
                lift.changeClawState(Lift.ClawStates.CLOSED);
                grabbedTime.reset();
            }

            if (lift.clawState == Lift.ClawStates.CLOSED && grabbedTime.seconds() > 0.3 && lift.liftState == Lift.LiftStates.COLLECT){
                lift.changeLiftState(Lift.LiftStates.GROUND);
            }

            /**Pole Detection*/
            if (lift.checkForPole())
                lift.swapToScore();
            else lift.swapFromScore();
        }
    }
}
