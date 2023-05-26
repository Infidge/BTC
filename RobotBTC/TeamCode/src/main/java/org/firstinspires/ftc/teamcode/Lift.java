package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

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

    DigitalChannel leftSwitch;
    DigitalChannel rightSwitch;

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
        COLLECT,
        GROUND,
        LOW,
        MID,
        HIGH,
    }

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
        leftSwitch = hwMap.get(DigitalChannel.class, "leftSwitch");
        rightSwitch = hwMap.get(DigitalChannel.class, "rightSwitch");
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
                //lift pid controller
                liftState = LiftStates.COLLECT;
                break;
            case GROUND:
                //lift pid controller
                liftState = LiftStates.GROUND;
                break;
            case LOW:
                //lift pid controller
                liftState = LiftStates.LOW;
                break;
            case MID:
                //lift pid controller
                liftState = LiftStates.MID;
                break;
            case HIGH:
                //lift pid controller
                liftState = LiftStates.HIGH;
                break;
            default:
                break;
        }
    }

    public void readSensors(SensorReading clawReading, SensorReading guideReading){
        clawReading.distance = clawSensor.getDistance(DistanceUnit.CM);
        clawReading.red = clawSensor.red();
        clawReading.green = clawSensor.green();
        clawReading.blue = clawSensor.blue();

        guideReading.distance = guideSensor.getDistance(DistanceUnit.CM);
        guideReading.red = guideSensor.red();
        guideReading.green = guideSensor.green();
        guideReading.blue = guideSensor.blue();
    }

    public boolean checkForCone(SensorReading clawReading){
        if (clawReading.distance < 3.0 && (Math.max(clawReading.red, Math.max(clawReading.green, clawReading.blue)) == clawReading.red || Math.max(clawReading.red, Math.max(clawReading.green, clawReading.blue)) == clawReading.blue) && clawState == ClawStates.OPEN)
            return true;
        else return false;
    }

    public boolean checkForPole(SensorReading guideReading){
        if (guideReading.distance < 3.0 && clawState == ClawStates.CLOSED && guideState == GuideStates.UP /*&& yellow detection*/)
            return true;
        else return false;
    }
}