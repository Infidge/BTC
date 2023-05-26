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

        /**initPositions*/

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
                break;
            case SCORE:
                armLeft.setPosition(Constants.leftArmScore);
                armRight.setPosition(Constants.rightArmScore);
                break;
            default:
                break;
        }
    }

    public void changeGuideState(GuideStates state){
        switch (state) {
            case UP:
                poleGuide.setPosition(Constants.guideUp);
                break;
            case DOWN:
                poleGuide.setPosition(Constants.guideDown);
                break;
            default:
                break;
        }
    }

    public void changeClawState(ClawStates state){
        switch (state) {
            case OPEN:
                claw.setPosition(Constants.openClaw);
                break;
            case CLOSED:
                claw.setPosition(Constants.closedClaw);
                break;
            default:
                break;
        }
    }

    public void changeUprightStickState(UprightStickStates state){
        switch (state) {
            case UP:
                uprightStick.setPosition(Constants.uprightStickUp);
                break;
            case DOWN:
                uprightStick.setPosition(Constants.uprightStickDown);
                break;
            default:
                break;
        }
    }

    public void changeLiftState(LiftStates state){
        switch (state) {
            case COLLECT:
                //lift pid controller
                break;
            case GROUND:
                //lift pid controller
                break;
            case LOW:
                //lift pid controller
                break;
            case MID:
                //lift pid controller
                break;
            case HIGH:
                //lift pid controller
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

}