package org.firstinspires.ftc.teamcode.Autonomous;

/*import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;*/
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetection.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Lift;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Auto_Right", group = "Baciu")
public class Auto_Right extends LinearOpMode {

    private enum FSMStages {
        scanAprilTag,
        scorePreLoad,
        goToCone1,
        scoreCone1,
        goToCone2,
        scoreCone2,
        goToCone3,
        scoreCone3,
        goToCone4,
        scoreCone4,
        goToCone5,
        scoreCone5,
        goToPark,
        stopRobot
    }

    private final FSMStages[] fsmStages = FSMStages.values();
    private int fsmStage = 0;

    Drivetrain dt = new Drivetrain();
    Lift lift = new Lift();

    ElapsedTime detectionTime = new ElapsedTime();
    ElapsedTime sensorReadTime =  new ElapsedTime();
    ElapsedTime liftReadTime =  new ElapsedTime();
    ElapsedTime switchReadTime =  new ElapsedTime();

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {

        dt.initDrivetrain(hardwareMap);
        lift.initLift(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        double tagSize = 0.166;
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        boolean tagFound = false;
        int parkZone = 2;

        waitForStart();
        detectionTime.reset();

        while (opModeIsActive()) {
            drive.update();
            lift.PIDController();

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

            switch (fsmStages[fsmStage]) {
                case scanAprilTag:
                    while (!tagFound && detectionTime.seconds() < 0.2) {
                        ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getLatestDetections();

                        if (detections.size() != 0) {
                            for (AprilTagDetection tag : detections) {
                                if (tag.id == 13) {
                                    parkZone = 1;
                                    tagFound = true;
                                    break;
                                } else if (tag.id == 12) {
                                    parkZone = 3;
                                    tagFound = true;
                                    break;
                                } else if (tag.id == 19) {
                                    tagFound = true;
                                    break;
                                }
                            }
                        }
                    }
                    if (parkZone == 1);
                    else if (parkZone == 3);
                    else;
                    fsmStage++;
                    break;
                case scorePreLoad:
                    lift.changeLiftState(Lift.LiftStates.HIGH);
                    lift.changeArmState(Lift.ArmStates.SCORE);
                    lift.changeGuideState(Lift.GuideStates.UP);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case goToCone1:
                    lift.changeLiftState(Lift.LiftStates.CONE1);
                    lift.changeArmState(Lift.ArmStates.COLLECT);
                    lift.changeGuideState(Lift.GuideStates.DOWN);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case scoreCone1:
                    lift.changeLiftState(Lift.LiftStates.HIGH);
                    lift.changeArmState(Lift.ArmStates.SCORE);
                    lift.changeGuideState(Lift.GuideStates.UP);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case goToCone2:
                    lift.changeLiftState(Lift.LiftStates.CONE2);
                    lift.changeArmState(Lift.ArmStates.COLLECT);
                    lift.changeGuideState(Lift.GuideStates.DOWN);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case scoreCone2:
                    lift.changeLiftState(Lift.LiftStates.HIGH);
                    lift.changeArmState(Lift.ArmStates.SCORE);
                    lift.changeGuideState(Lift.GuideStates.UP);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case goToCone3:
                    lift.changeLiftState(Lift.LiftStates.CONE3);
                    lift.changeArmState(Lift.ArmStates.COLLECT);
                    lift.changeGuideState(Lift.GuideStates.DOWN);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case scoreCone3:
                    lift.changeLiftState(Lift.LiftStates.HIGH);
                    lift.changeArmState(Lift.ArmStates.SCORE);
                    lift.changeGuideState(Lift.GuideStates.UP);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case goToCone4:
                    lift.changeLiftState(Lift.LiftStates.CONE4);
                    lift.changeArmState(Lift.ArmStates.COLLECT);
                    lift.changeGuideState(Lift.GuideStates.DOWN);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case scoreCone4:
                    lift.changeLiftState(Lift.LiftStates.HIGH);
                    lift.changeArmState(Lift.ArmStates.SCORE);
                    lift.changeGuideState(Lift.GuideStates.UP);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case goToCone5:
                    lift.changeLiftState(Lift.LiftStates.CONE5);
                    lift.changeArmState(Lift.ArmStates.COLLECT);
                    lift.changeGuideState(Lift.GuideStates.DOWN);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case scoreCone5:
                    lift.changeLiftState(Lift.LiftStates.HIGH);
                    lift.changeArmState(Lift.ArmStates.SCORE);
                    lift.changeGuideState(Lift.GuideStates.UP);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case goToPark:
                    lift.changeLiftState(Lift.LiftStates.COLLECT);
                    lift.changeArmState(Lift.ArmStates.COLLECT);
                    lift.changeGuideState(Lift.GuideStates.DOWN);
                    if (!drive.isBusy()) fsmStage++;
                    break;
                case stopRobot:
                    dt.stopDrivetrain();
                    lift.stopLift();
                    fsmStage++;
                    break;
                default:
                    break;
            }
        }
    }
}