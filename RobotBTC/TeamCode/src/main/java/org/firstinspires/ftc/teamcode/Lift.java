package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Util.Constants;
import org.firstinspires.ftc.teamcode.Util.LimitSwitch;
import org.firstinspires.ftc.teamcode.Util.SensorReading;

public class Lift {

    DcMotorEx liftLeft;
    DcMotorEx liftRight;

    Servo armLeft;
    Servo armRight;
    Servo claw;
    Servo poleGuide;
    Servo uprightStick;

    RevColorSensorV3 clawSensor;
    RevColorSensorV3 guideSensor;

    DigitalChannel switchL;
    DigitalChannel switchR;

    LimitSwitch leftSwitch = new LimitSwitch(switchL, "leftSwitch");
    LimitSwitch rightSwitch = new LimitSwitch(switchR, "rightSwitch");

    SensorReading clawReadings = new SensorReading();
    SensorReading guideReadings = new SensorReading();

    public enum ArmStates{
        COLLECT,
        SCORE
    }

    public enum GuideStates{
        UP,
        DOWN
    }

    public enum ClawStates{
        OPEN,
        CLOSED
    }

    public enum UprightStickStates{
        UP,
        DOWN
    }

    public enum LiftStates{
        COLLECT (Constants.liftCollect),
        GROUND (Constants.liftGroundJunction),
        LOW (Constants.liftLowJunction),
        SCORE_LOW (Constants.liftScoreLowJunction),
        MID (Constants.liftMidJunction),
        SCORE_MID (Constants.liftScoreMidJunction),
        HIGH (Constants.liftHighJunction),
        SCORE_HIGH (Constants.liftScoreHighJunction);

        private int liftPos;

        LiftStates(int liftPos) {
            this.liftPos = liftPos;
        }

        public int get(){
            return liftPos;
        }
    }

    public int lastError = 0, currError = 0, errorSum = 0;
    public double kP = 0.0, kI = 0.0, kD = 0.0;
    public int liftDirectionCoeff = 1;
    public int errorCount = 0;

    public int leftLiftPos, rightLiftPos;

    ArmStates armState = ArmStates.COLLECT;
    GuideStates guideState = GuideStates.DOWN;
    ClawStates clawState = ClawStates.CLOSED;
    UprightStickStates uprightStickState = UprightStickStates.UP;
    LiftStates liftState = LiftStates.COLLECT;

    public Lift(){
    }

    public void initLift(HardwareMap hwMap){

        //Motors
        liftLeft = hwMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hwMap.get(DcMotorEx.class, "liftRight");

        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftLeft.setPower(0.0);
        liftRight.setPower(0.0);

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftRight.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servos
        armLeft = hwMap.get(Servo.class, "armLeft");
        armRight = hwMap.get(Servo.class, "armRight");
        claw = hwMap.get(Servo.class, "claw");
        poleGuide = hwMap.get(Servo.class, "poleGuide");
        uprightStick = hwMap.get(Servo.class, "uprightStick");

        armLeft.setPosition(Constants.leftArmCollect);
        armRight.setPosition(Constants.rightArmCollect);
        claw.setPosition(Constants.closedClaw);
        poleGuide.setPosition(Constants.guideDown);
        uprightStick.setPosition(Constants.uprightStickUp);

        //Sensors
        clawSensor = hwMap.get(RevColorSensorV3.class, "clawSensor");
        guideSensor = hwMap.get(RevColorSensorV3.class, "guideSensor");

        //LimitSwitches
        leftSwitch.digitalSwitch = hwMap.get(DigitalChannel.class, leftSwitch.switchName);
        rightSwitch.digitalSwitch = hwMap.get(DigitalChannel.class, rightSwitch.switchName);
    }


    public void changeArmState(ArmStates state){
        switch (state) {
            case COLLECT:
                armLeft.setPosition(Constants.leftArmCollect);
                armRight.setPosition(Constants.rightArmCollect);
                armState = ArmStates.COLLECT;
                break;
            case SCORE:
                armLeft.setPosition(Constants.leftArmScore);
                armRight.setPosition(Constants.rightArmScore);
                armState = ArmStates.SCORE;
                break;
            default:
                break;
        }
    }

    public void toggleArmState(){
        if (armState == ArmStates.COLLECT)
            changeArmState(ArmStates.SCORE);
        else changeArmState(ArmStates.COLLECT);
    }

    public void changeGuideState(GuideStates state){
        switch (state) {
            case UP:
                poleGuide.setPosition(Constants.guideUp);
                guideState = GuideStates.UP;
                break;
            case DOWN:
                poleGuide.setPosition(Constants.guideDown);
                guideState = GuideStates.DOWN;
                break;
            default:
                break;
        }
    }

    public void toggleGuideState(){
        if (guideState == GuideStates.UP)
            changeGuideState(GuideStates.DOWN);
        else changeGuideState(GuideStates.UP);
    }

    public void changeClawState(ClawStates state){
        switch (state) {
            case OPEN:
                claw.setPosition(Constants.openClaw);
                clawState = ClawStates.OPEN;
                break;
            case CLOSED:
                claw.setPosition(Constants.closedClaw);
                clawState = ClawStates.CLOSED;
                break;
            default:
                break;
        }
    }

    public void toggleClawState(){
        if (clawState == ClawStates.OPEN)
            changeClawState(ClawStates.CLOSED);
        else changeClawState(ClawStates.OPEN);
    }

    public void changeUprightStickState(UprightStickStates state){
        switch (state) {
            case UP:
                uprightStick.setPosition(Constants.uprightStickUp);
                uprightStickState = UprightStickStates.UP;
                break;
            case DOWN:
                uprightStick.setPosition(Constants.uprightStickDown);
                uprightStickState = UprightStickStates.DOWN;
                break;
            default:
                break;
        }
    }

    public void toggleUprightStickState(){
        if (uprightStickState == UprightStickStates.UP)
            changeUprightStickState(UprightStickStates.DOWN);
        else changeUprightStickState(UprightStickStates.UP);
    }

    public void changeLiftState(LiftStates state){
        switch (state) {
            case COLLECT:
                liftState = LiftStates.COLLECT;
                errorCount = 0;
                errorSum = 0;
                break;
            case GROUND:
                liftState = LiftStates.GROUND;
                errorCount = 0;
                errorSum = 0;
                break;
            case LOW:
                liftState = LiftStates.LOW;
                errorCount = 0;
                errorSum = 0;
                break;
            case SCORE_LOW:
                liftState = LiftStates.SCORE_LOW;
                errorCount = 0;
                errorSum = 0;
                break;
            case MID:
                liftState = LiftStates.MID;
                errorCount = 0;
                errorSum = 0;
                break;
            case SCORE_MID:
                liftState = LiftStates.SCORE_MID;
                errorCount = 0;
                errorSum = 0;
                break;
            case HIGH:
                liftState = LiftStates.HIGH;
                errorCount = 0;
                errorSum = 0;
                break;
            case SCORE_HIGH:
                liftState = LiftStates.SCORE_HIGH;
                errorCount = 0;
                errorSum = 50;
                break;
            default:
                break;
        }
    }

    public void PIDController(){
        double liftPower = 0.0;
        if (liftState!=LiftStates.COLLECT){
            if (liftState.get() < leftLiftPos)
                liftDirectionCoeff = -1;
            else liftDirectionCoeff = 1;
            errorCount += 1;
            currError = Math.abs(liftState.get() - leftLiftPos);
            errorSum += currError;
            liftPower = liftDirectionCoeff * (kP * currError + kD * (currError - lastError) + kI * errorSum / errorCount);
            liftPower = Range.clip(liftPower, -1.0, 1.0);
        }
        else if (!leftSwitch.isPressed() || !rightSwitch.isPressed()){
            liftPower = -Constants.liftDefaultPower;
        }
        else if (leftSwitch.isPressed() || rightSwitch.isPressed()){
            liftPower = 0.0;
            liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        liftLeft.setPower(liftPower);
        liftRight.setPower(liftPower);
    }
    
    public void swapToScore(){
        if (liftState == LiftStates.LOW)
            changeLiftState(LiftStates.SCORE_LOW);
        else if (liftState == LiftStates.MID)
            changeLiftState(LiftStates.SCORE_MID);
        else if (liftState == LiftStates.HIGH)
            changeLiftState(LiftStates.SCORE_HIGH);
    }
    
    public void swapFromScore(){
        if (liftState == LiftStates.SCORE_LOW)
            changeLiftState(LiftStates.LOW);
        else if (liftState == LiftStates.SCORE_MID)
            changeLiftState(LiftStates.MID);
        else if (liftState == LiftStates.SCORE_HIGH)
            changeLiftState(LiftStates.HIGH);
    }


    public void readSensors(){
        clawReadings.distance = clawSensor.getDistance(DistanceUnit.CM);
        clawReadings.red = clawSensor.red();
        clawReadings.green = clawSensor.green();
        clawReadings.blue = clawSensor.blue();

        guideReadings.distance = guideSensor.getDistance(DistanceUnit.CM);
        guideReadings.red = guideSensor.red();
        guideReadings.green = guideSensor.green();
        guideReadings.blue = guideSensor.blue();
    }

    public void readLiftEncoders(){
        leftLiftPos = liftLeft.getCurrentPosition();
        rightLiftPos = liftRight.getCurrentPosition();
    }

    public void readSwitches(){
        leftSwitch.switchState = switchL.getState();
        rightSwitch.switchState = switchR.getState();
    }

    public boolean checkForCone(){
        if (clawReadings.distance < 3.0 && (Math.max(clawReadings.red, Math.max(clawReadings.green, clawReadings.blue)) == clawReadings.red || Math.max(clawReadings.red, Math.max(clawReadings.green, clawReadings.blue)) == clawReadings.blue) && clawState == ClawStates.OPEN  && liftState == Lift.LiftStates.COLLECT)
            return true;
        else return false;
    }

    public boolean checkForPole(){
        if (guideReadings.distance < 3.0 /*&& yellow pole detection*/ && clawState == ClawStates.CLOSED && guideState == GuideStates.UP && liftState != Lift.LiftStates.COLLECT && liftState != Lift.LiftStates.GROUND)
            return true;
        else return false;
    }
}