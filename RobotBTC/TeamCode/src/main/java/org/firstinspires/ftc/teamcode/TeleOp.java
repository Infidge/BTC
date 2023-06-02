/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Robot: Teleop POV", group="Robot")
public class TeleOp extends LinearOpMode {

    Drivetrain dt = new Drivetrain();
    Lift lift = new Lift();
    SensorReading clawReadings = new SensorReading();
    SensorReading guideReadings = new SensorReading();
    ElapsedTime sensorReadTime =  new ElapsedTime();

    @Override
    public void runOpMode() {

        waitForStart();
        sensorReadTime.reset();

        while (opModeIsActive()) {

            dt.initDrivetrain(hardwareMap);
            lift.initLift(hardwareMap);

            if (sensorReadTime.milliseconds() > 75) {
                lift.readSensors(clawReadings, guideReadings);
                sensorReadTime.reset();
            }

            dt.mecanumDrive(gamepad1);

            if (gamepad1.a)
                lift.toggleUprightStickState();

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

            lift.PIDController();

            if (gamepad2.a)
                lift.toggleClawState();

            if (gamepad1.b)
                lift.toggleArmState();

            if (gamepad2.x)
                lift.toggleGuideState();

            if (lift.checkForCone(clawReadings) && lift.liftState == Lift.LiftStates.COLLECT) {
                lift.changeClawState(Lift.ClawStates.CLOSED);
                lift.changeLiftState(Lift.LiftStates.GROUND);
            }

            if (lift.checkForPole(guideReadings) && lift.liftState != Lift.LiftStates.COLLECT && lift.liftState != Lift.LiftStates.GROUND)
                lift.swapToScore();
            else lift.swapFromScore();

            sleep(10);
        }
    }
}
