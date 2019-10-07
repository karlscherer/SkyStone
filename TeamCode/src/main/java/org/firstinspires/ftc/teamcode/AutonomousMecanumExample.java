/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name ="AutonomousMecanumExample")

// @Disabled

public class AutonomousMecanumExample extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private Servo servoArm = null;

    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY = "ATqulq//////AAABmfYPXE+z1EORrVmv4Ppo3CcPktGk5mvdMnvPi9/T3DMYGc2mju8KUyG9gAB7pKlb9k9SZnM0YSq1JUZ6trE1ZKmMU8z5QPuhA/b6/Enb+XVGwmjrRjhMfNtUNgiZDhtsUvxr9fQP4HVjTzlz4pv0z3MeWZmkAgIN8T8YM0EFWrW4ODqYQmZjB0Nri2KKVM9dlOZ5udPfTZ9YvMgrCyxxG7O8P84AvwCAyXxzxelL4OfGnbygs0V60CQHx51gqrki613PT/9D1Q1io5+UbN6xAQ26AdYOTmADgJUGlfC2eMyqls4qAIoOj+pcJbm5ryF5yW9pEGHmvor1c9HlCFwhKxiaxw+cTu8AEaAdNuR65i/p";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private static final double ticks = 1440; // AndyMark = 1120, Tetrix = 1440
    private static final double gearReduction = 1.0; // Greater than 1.0; Less than 1.0 if geared up
    private static final double wheelDiameterInches = 4.0;
    private static final double pi = 3.1415;
    private static final double conversionTicksToInches = (ticks * gearReduction) / (pi * wheelDiameterInches);

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backRight = hardwareMap.dcMotor.get("backRight");

        servoArm = hardwareMap.servo.get("servoArm");

        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope) navxMicro;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        servoArm.setPosition(0);

        StopAndResetEncoder();
        RunUsingEncoder();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        telemetry.log().add("Gyro Calibrating. Do Not Move!");

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (navxMicro.isCalibrating()) {
            telemetry.addData("Calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
            telemetry.update();
            Thread.sleep(50);
        }

        telemetry.log().clear();
        telemetry.update();
        telemetry.log().add("Gyro Calibrated. Press Play to Begin!");

        waitForStart();
        telemetry.clear();
        telemetry.update();

//        if (opModeIsActive()) {
//            while (opModeIsActive()) {
//                if (tfod != null) {
//                    // getUpdatedRecognitions() will return null if no new information is available since
//                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//                    if (updatedRecognitions != null) {
//                        telemetry.addData("# Object Detected", updatedRecognitions.size());
//
//                        // step through the list of recognitions and display boundary info.
//                        int i = 0;
//                        for (Recognition recognition : updatedRecognitions) {
//                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                                    recognition.getLeft(), recognition.getTop());
//                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                                    recognition.getRight(), recognition.getBottom());
//                        }
//                        telemetry.update();
//                    }
//                }
//            }
//        }
//
//        if (tfod != null) {
//            tfod.shutdown();
//        }

        // =========================================================================================
        // Start writing the movements for this autonomous sequence below:
        // =========================================================================================

        GyroDrive(4000, .50,0,1000);
//        GyroDrive(-36,-.50,0,1000);
//        GyroRight(.50,75,500);
//        GyroStrafe(-36, -.50,90,1000);
//        GyroStrafe(36,.50,90,1000);
//        GyroLeft(.50,-75,1000);
//        GyroRight(.50,-15,1000);

        // =========================================================================================
    }

    public void GyroDrive(int distance, double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            StopAndResetEncoder();
            RunUsingEncoder();
            DriveByInches(distance);
            RunToPosition();
            while (opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (-gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(power + error);
                    backLeft.setPower(power - error);
                    backRight.setPower(power + error);
                    DisplayTelemetry();
                } else{
                    StopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void GyroStrafe(int distance, double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            StopAndResetEncoder();
            RunUsingEncoder();
            StrafeByInches(distance);
            RunToPosition();
            while (opModeIsActive()){
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double errorMultiple = 1.0;
                double error = (errorMultiple * (-gyroHeading - angle) / 100);
                if (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                    frontLeft.setPower(power - error);
                    frontRight.setPower(-power + error);
                    backLeft.setPower(-power - error);
                    backRight.setPower(power + error);
                    DisplayTelemetry();
                } else {
                    StopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void GyroRight(double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            StopAndResetEncoder();
            RunUsingEncoder();
            while (opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double invertGyroHeading = gyroHeading * -1;
                if (invertGyroHeading < angle) {
                    frontLeft.setPower(power);
                    frontRight.setPower(-power);
                    backLeft.setPower(power);
                    backRight.setPower(-power);
                    DisplayTelemetry();
                }else{
                    StopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void GyroLeft(double power, double angle, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            StopAndResetEncoder();
            RunUsingEncoder();
            while (opModeIsActive()) {
                double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double invertGyroHeading = gyroHeading * -1;
                if (invertGyroHeading > angle) {
                    frontLeft.setPower(-power);
                    frontRight.setPower(power);
                    backLeft.setPower(-power);
                    backRight.setPower(power);
                    DisplayTelemetry();
                }else{
                    StopDriving();
                    Thread.sleep(pause);
                    break;
                }
            }
        }
    }

    public void StopDriving() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void DriveByInches(int distance) {
        frontLeft.setTargetPosition(distance * (int) conversionTicksToInches);
        frontRight.setTargetPosition(distance * (int) conversionTicksToInches);
        backLeft.setTargetPosition(distance * (int) conversionTicksToInches);
        backRight.setTargetPosition(distance * (int) conversionTicksToInches);
    }

    public void StrafeByInches(int distance) {
        frontLeft.setTargetPosition(distance * (int) conversionTicksToInches);
        frontRight.setTargetPosition(-distance * (int) conversionTicksToInches);
        backLeft.setTargetPosition(-distance * (int) conversionTicksToInches);
        backRight.setTargetPosition(distance * (int) conversionTicksToInches);
    }

    public void TurnByInches(int distance) {
        frontLeft.setTargetPosition(distance * (int) conversionTicksToInches);
        frontRight.setTargetPosition(-distance * (int) conversionTicksToInches);
        backLeft.setTargetPosition(distance * (int) conversionTicksToInches);
        backRight.setTargetPosition(-distance * (int) conversionTicksToInches);
    }

    public void StopAndResetEncoder() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RunWithoutEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void RunUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void RunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void DisplayTelemetry() {
        double gyroHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double invertGyroHeading = gyroHeading * -1;
        double errorMultiple = 1.0;
        double error = (errorMultiple * (-gyroHeading - 0) / 100);
        double frontLeftInches = frontLeft.getCurrentPosition() / conversionTicksToInches;
        double frontRightInches = frontRight.getCurrentPosition() / conversionTicksToInches;
        double backLeftInches = backLeft.getCurrentPosition() / conversionTicksToInches;
        double backRightInches = backRight.getCurrentPosition() / conversionTicksToInches;

        telemetry.clear();
        telemetry.addLine()
                .addData("Heading: ", "%.1f", invertGyroHeading);
        telemetry.addLine()
                .addData("Front Left Inches ",(int)frontLeftInches + "   Power: " + "%.1f", frontLeft.getPower());
        telemetry.addLine()
                .addData("Front Right Inches: ",(int)frontRightInches + "   Power: " + "%.1f", frontRight.getPower());
        telemetry.addLine()
                .addData("Back Left Inches: ",(int)backLeftInches + "   Power: " + "%.1f", backLeft.getPower());
        telemetry.addLine()
                .addData("Back Right Inches: ",(int)backRightInches + "   Power: " + "%.1f", backRight.getPower());
        telemetry.update();
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
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

//==================================================================================================
    // Use these if you don't have a gyro, but have connected encoders.

    public void Drive(int distance, double power, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            StopAndResetEncoder();
            RunUsingEncoder();
            DriveByInches(distance);
            RunToPosition();
            Power(power);
            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                DisplayTelemetry();
            }
            StopDriving();
            Thread.sleep(pause);
        }
    }

    public void Strafe(int distance, double power, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            StopAndResetEncoder();
            StrafeByInches(distance);
            RunToPosition();
            Power(power);
            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                DisplayTelemetry();
            }
            StopDriving();
            RunUsingEncoder();
            Thread.sleep(pause);
        }
    }

    public void Turn(int distance, double power, int pause) throws InterruptedException {
        if (opModeIsActive()) {
            StopAndResetEncoder();
            RunUsingEncoder();
            TurnByInches(distance);
            RunToPosition();
            Power(power);
            while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()) {
                DisplayTelemetry();
            }
            StopDriving();
            Thread.sleep(pause);
        }
    }

    public void Power(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

//==================================================================================================
    // Use these if you don't have a gyro or encoder wires.
    // AndyMark and Tetrix have built in encoders.
    // The wires for each brand are different and cannot be switched without alteration.

    public void DriveByTime(int time, int power){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(time);
    }

    public void StrafeByTime(int time, int power){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(time);
    }

    public void TurnByTime(int time, int power){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(time);
    }
}