package org.firstinspires.ftc.teamcode.ftc14657;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;


public class Auto14657_Base_UG extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();

    Hardware14657_UG robot   = new Hardware14657_UG();
    SampleMecanumDrive drive = null;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    static final String VUFORIA_KEY = "ATJlfF//////AAABmU9HdbHCyUhQiqrgbfIP3+pHG90Y4MIDo0mEG" +
            "k7ec6BdBVYehjQKhCAGZck06Dlvg92a0yowpYAdAkROXZ9" +
            "OgaTZzf90cHTw3WTO6kIDA6l0i2HHv7amwBa9NThgKVCEP" +
            "cyoNkEK8KdSEB34IgJ0S/T1UP+amGRYKKusKCMWvM7WNyvv" +
            "L2fe+LSkcMeAgYDzHvWIIyneel/UtiQ5M3MQY9BZQmpOQRXY" +
            "PfrcAYjdpdbrRy15jT/8YptFRd/8vdV/6HlPLk0o/nz+/jzDA" +
            "6eaRlMg9n3yB+m6ZqgVs+pVEO0hYXhaV5eQkAQcDbHQtYQFuQ" +
            "mXV9Xl5VdHy/L/q/Q3cgKQNUeSDF9lzIJ/SNEaYy72";

    VuforiaLocalizer vuforia;
    TFObjectDetector tfod;

    //    double adjTimeout = 0.2;
    int starterStackNo = 0;
//    int Velocity_Powershot = 1600;
    int Velocity_Powershot = 1550;
//    int Velocity_HighGoal = 2300;
    int Velocity_HighGoal = 2400;
//    int Velocity_HighGoal_middle = 1900;
    int Velocity_HighGoal_middle = 2400;
//    int Velocity_Powershot = 3400;
//    int Velocity_HighGoal = 4900;

    int Velocity_HighGoal_far = 2800;

//    int Velocity_Collection = 2000;
    int Velocity_Collection = 2200;
    int Velocity_Collection_Powershot = 1200;

    VoltageSensor batteryVoltageSensor;
    double fullPowerVoltage = 12.95;

    public void init(HardwareMap ahwMap) {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        // set initial position
//        openServo_Grab(1000);
        closeServo_Grab(1000);
        closeServo_Grab2(100);
        runServo_TriggerOpen(500);
        robot.shooter.setPower(0);
        robot.collection.setPower(0);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        // Gyro sensor setup
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
//        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
//        // and named "imu".
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//
//        imu.initialize(parameters);
//
//
//
//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !imu.isGyroCalibrated())
//        {
//            sleep(50);
//            idle();
//        }
//        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
//        telemetry.update();
//
//        startAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        gAngle = 0;



        detectStarterStack();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    public void detectStarterStack(){

        //Turn the flash on
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);

        //The pre-placed stacks of Rings will be adjusted to either zero (0), one (1), or four (4),
        //corresponding to the selected Target Zone Goal (A, B, or C).
        starterStackNo = 0;
        while ( !isStopRequested() && !isStarted() ) {
            //        while (!isStopRequested() && starterStackNo == 0) {

            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                //make sure ONE object detected
                if (updatedRecognitions != null && updatedRecognitions.size() == 1) {
                    //                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());

                        RobotLog.d(String.format("ULTIMATE_DEBUG - detectStarterStack() -label (%d)", i), recognition.getLabel());
                        RobotLog.d(String.format("ULTIMATE_DEBUG - detectStarterStack() -left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        RobotLog.d(String.format("ULTIMATE_DEBUG - detectStarterStack() -right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        //                        if (recognition.getLabel().equalsIgnoreCase("SINGLE")) {
                        int ringHeight_Mid = 140;
                        int ringWidth_Mid = 140;
                        int ringWidth_Max = 220;

                        float ringHeight = Math.abs(recognition.getRight() - recognition.getLeft());
                        float ringWidth = Math.abs(recognition.getBottom() - recognition.getTop());

                        telemetry.addData(String.format(" Height: "), ringHeight);
                        telemetry.addData(String.format(" Width: "), ringWidth);
                        if( ringWidth > ringWidth_Mid && ringWidth <= ringWidth_Max) {
                            telemetry.addData(String.format("GOOD Width: "), ringWidth);
                            if (ringHeight < ringHeight_Mid) {
                                starterStackNo = 1;
                                telemetry.addData(String.format("SINGLE - Height: "), ringHeight);
                            } else {
                                starterStackNo = 4;
                                telemetry.addData(String.format("QUAD - Height: "), ringHeight);
                            }
                        } else {
                            starterStackNo = 0;
                            telemetry.addData(String.format(" Width - noise  "), ringWidth);
                        }
                    }
                    telemetry.addData(String.format("FINAL  starterStackNo "), starterStackNo);
                    telemetry.update();
                } else {
                    starterStackNo = 0;
                    telemetry.addData("# 0 Object Detected", starterStackNo);
                    telemetry.update();
                }
            }

            //            telemetry.addData(String.format("  starterStackNo: "), starterStackNo);
            //            telemetry.update();
            sleep(500);
            //            idle();
        }

        //        telemetry.addData(String.format("FINAL2  starterStackNo "), starterStackNo);
        //        telemetry.update();
        RobotLog.d("ULTIMATE_DEBUG - detectStarterStack() -starterStackNo:" + starterStackNo);

        //Turn the flash off
        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(false);

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {


    }


    public void closeServo_Grab (long sleeptimeMiliSec) {

//        robot.grab.setPosition(0.4);
        robot.grab.setPosition(0);
        sleep(sleeptimeMiliSec);

    }

    public void openServo_Grab (long sleeptimeMiliSec, boolean fullOpen) {

//        robot.grab.setPosition(1);
        if(fullOpen) {
            robot.grab.setPosition(0.9);

        } else {
            robot.grab.setPosition(0.8);
        }
        sleep(sleeptimeMiliSec);

    }

    public void closeServo_Grab2 (long sleeptimeMiliSec) {

        robot.grab2.setPosition(robot.grab2_closePos);
        sleep(sleeptimeMiliSec);
    }

    public void openServo_Grab2 (long sleeptimeMiliSec) {

        robot.grab2.setPosition(robot.grab2_openPos);
        sleep(sleeptimeMiliSec);
    }



    public void runMotor_LiftCollect (long sleeptimeMiliSec) {

        robot.lift.setPower(-1);
        sleep(sleeptimeMiliSec);
    }

    public void runMotor_LiftCollectRevert (long sleeptimeMiliSec) {

        robot.lift.setPower(1);
        sleep(sleeptimeMiliSec);
    }


    public void runMotor_DropWobble (long sleeptimeMiliSec) {

        // robot.wobble.setPower(0.5);
//        robot.wobble.setPower(0.75);
        robot.wobble.setPower(0.65);
        sleep(sleeptimeMiliSec);
        robot.wobble.setPower(0);
    }

    public void runMotor_LiftWobble (long sleeptimeMiliSec) {

        robot.wobble.setPower(-0.85);
        sleep(sleeptimeMiliSec);
        robot.wobble.setPower(0);
//            onFlag_LiftWobble = false;
    }

    public void runServo_DropWobble2 (long sleeptimeMiliSec) {

        robot.wobble2.setPosition(robot.wobble2_dropPos);
        sleep(sleeptimeMiliSec);

    }

    public void runServo_LiftWobble2 (long sleeptimeMiliSec) {

        robot.wobble2.setPosition(robot.wobble2_liftPos);
        sleep(sleeptimeMiliSec);

    }

    public void runServo_TriggerClose (long sleeptimeMiliSec){
        robot.trigger.setPosition(robot.trigger_closePos);
        sleep(sleeptimeMiliSec);
    }

    public void runServo_TriggerOpen (long sleeptimeMiliSec){
        robot.trigger.setPosition(robot.trigger_openPos);
        sleep(sleeptimeMiliSec);
    }

//    public void runServo_FlipDown (long sleeptimeMiliSec){
//
//        robot.flip.setPower(-1);
//        sleep(sleeptimeMiliSec);
//        if(sleeptimeMiliSec>0){
//            robot.flip.setPower(0);
//        }
//    }
//    public void runServo_FlipUp (long sleeptimeMiliSec){
//
//        robot.flip.setPower(1);
//        sleep(sleeptimeMiliSec);
//         if(sleeptimeMiliSec>0){
//             robot.flip.setPower(0);
//         }
//    }
//
//    public void runServo_FlipReset (long sleeptimeMiliSec){
//
//        robot.flip.setPower(0);
//        sleep(sleeptimeMiliSec);
//        //        robot.flip.setPower(0);
//    }

}

class ShooterWheelThread extends Thread{
    Hardware14657_UG robot   = null;
    double rotationSpeed    = 0;
    public ShooterWheelThread(Hardware14657_UG robot, double rotationSpeed) {
        this.robot = robot;
        this.rotationSpeed = rotationSpeed;
        this.setName("ShooterWheelThread");
        RobotLog.d("ULTIMATE_DEBUG - ShooterWheelThread()" + this.getName());
    }

    @Override
    public void run() {
        try
        {

            while (!isInterrupted())
            {
                // we record the Y values in the main class to make showing them in telemetry
                // easier.

//                robot.shooter.setPower(rotationSpeed);
//                robot.shooter.setVelocity(rotationSpeed);
                robot.setVelocity(robot.shooter, rotationSpeed);

                sleep(1);


            }
        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        catch (InterruptedException e) {
            RobotLog.d("interrupted" + this.getName());
        }
        // an error occurred in the run loop.
        catch (Exception e) {
            RobotLog.d("interrupted" + this.getName());
        }

        RobotLog.d("end of thread " + this.getName());
    }
}

class CollectionThread extends Thread{
    Hardware14657_UG robot   = null;
    double rotationSpeed = 0;
    public CollectionThread(Hardware14657_UG robot, double ratationSpeed) {
        this.robot = robot;
        this.rotationSpeed = ratationSpeed;
        this.setName("CollectionThread");
        RobotLog.d("ULTIMATE_DEBUG - CollectionThread()" + this.getName());
    }

    @Override
    public void run() {
        try
        {

            while (!isInterrupted())
            {
                // we record the Y values in the main class to make showing them in telemetry
                // easier.

//                robot.collection.setPower(ratationSpeed);
                robot.collection.setVelocity(rotationSpeed);
                robot.lift.setPower(-1);
//                robot.flip.setPower(0);

                sleep(1);


            }
        }
        // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
        // or by the interrupted exception thrown from the sleep function.
        catch (InterruptedException e) {
            RobotLog.d("interrupted" + this.getName());
        }
        // an error occurred in the run loop.
        catch (Exception e) {
            RobotLog.d("interrupted" + this.getName());
        }

        RobotLog.d("end of thread " + this.getName());
    }
}


