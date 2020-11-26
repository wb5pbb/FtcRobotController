
    package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

    @TeleOp(name = "opmode_spin_steer_parallel_balance2 (Blocks to Java)", group = "")
    public class opmode_spin_steer_parallel_balance2 extends LinearOpMode {

        private Servo servoTest1;
        private Servo servoTest2;
        private Servo servoTest3;
        private Servo servoTest4;
        private Servo armServo1;
        private Servo gripServo1;
        private DcMotor motorTest1;
        private DcMotor motorTest2;
        private DcMotor motorTest3;
        private DcMotor motorTest4;

        BNO055IMU imu;

        // State used for updating telemetry
       Orientation angles;
       Acceleration gravity;

 /*  Set up the parameters with which we will use our IMU. Note that integration
                     algorithm here just reports accelerations to the logcat log; it doesn't actually
                     provide positional information. */

 /*       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

        //Set up our telemetry dashboard
        composeTelemetry();

        // Wait for start
        waitForStart();

        //start logging of measured acceleration and integration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
*/

        /**
         * This function is executed when this Op Mode is selected from the Driver Station.
         */
        @Override
        public void runOpMode() {



            double servo1zero;
            double servo2zero;
            double servo3zero;
            double servo4zero;
            double armServo1zero;
            double gripServo1zero;
            double deltaY;
            double deltaX;
            double oneeighty;
            double degree2servo;
            double ninety;
            double steeringDeadband;
            double parallelAngle;
            double steeringOn;
            double spinOn;
            double turnRadius;
            double alphaRight;
            double alphaLeft;
            double radiusFactor;
            double alpha1;
            double alpha2;
            double alpha3;
            double alpha4;
            double onefifty;
            double onethirty;
            double oneforty;
            double parallelOn;
            double tgt1speed;

            servoTest1 = hardwareMap.servo.get("servoTest1");
            servoTest2 = hardwareMap.servo.get("servoTest2");
            servoTest3 = hardwareMap.servo.get("servoTest3");
            servoTest4 = hardwareMap.servo.get("servoTest4");
            armServo1 = hardwareMap.servo.get("armServo1");
            gripServo1 = hardwareMap.servo.get("gripServo1");
            motorTest1 = hardwareMap.dcMotor.get("motorTest1");
            motorTest2 = hardwareMap.dcMotor.get("motorTest2");
            motorTest3 = hardwareMap.dcMotor.get("motorTest3");
            motorTest4 = hardwareMap.dcMotor.get("motorTest4");

            // Put initialization blocks here.
            waitForStart();
            // Setting the zero bias for each servo.  Zero is right direction. 90 is forward, 180 is left
            servo1zero = 0.03;
            servo2zero = 0.03;
            servo3zero = 0.03;
            servo4zero = 0.02;
            armServo1zero = 0.0;
            gripServo1zero = 0.0;
            // delta x and y are center of robot to wheel center in x, y coordinates.
            deltaY = 192.0;
            deltaX = 160.0;
            // set servo scaling value of 180 degrees rotation.
            oneeighty = 0.66;
            degree2servo = oneeighty / 180.0;
            oneforty = (140.0/180.0)* oneeighty;
            steeringDeadband = 0.005;
            servoTest1.setPosition(servo1zero + oneeighty);
            servoTest2.setPosition(servo2zero + 0.0);
            servoTest3.setPosition(servo3zero + oneeighty);
            servoTest4.setPosition(servo4zero + 0.0);
            armServo1.setPosition(armServo1zero);
            gripServo1.setPosition(gripServo1zero);
            parallelAngle = 0.0;
            steeringOn = 0.0;
            spinOn = 0.0;
            turnRadius = 0.0;
            alphaRight = 0.0;
            alphaLeft = 0.0;
            radiusFactor = 0.0;
            alpha1 = 0.0;
            alpha2 = 0.0;
            alpha3 = 0.0;
            alpha4 = 0.0;
            parallelOn = 0;
            steeringDeadband = 0.05;
            onefifty = (150.0 / 180.0) * oneeighty;
            onethirty = (130.0 / 180.0) * oneeighty;
            oneforty = (140.0 / 180.0) * oneeighty;
            ninety = (0.5) * oneeighty;
            telemetry.addData( "oneeighty", oneeighty );
            telemetry.addData( "oneforty", oneforty );
            telemetry.addData( "ninety", ninety );
            telemetry.update();
            if (opModeIsActive()) {
                // Put run blocks here.
                steeringDeadband = 0.05;
                while (opModeIsActive()) {
                    // Put loop blocks here.
                    if(gamepad1.left_trigger !=0) {
                        armServo1.setPosition(armServo1zero+ degree2servo*270.0*gamepad1.left_trigger);
                        gripServo1.setPosition(gripServo1zero + degree2servo*270.0*gamepad1.left_trigger);
                    }
                    if (gamepad1.y || parallelOn != 0) {
                        // Parallel Mode
                        parallelOn = 1;
                        if (parallelOn != 0) {
                            if (Math.abs(gamepad1.left_stick_y) >= steeringDeadband || Math.abs(gamepad1.left_stick_x) >= steeringDeadband) {
                                parallelAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180.0;
                                if (gamepad1.left_stick_y > steeringDeadband) {
                                    parallelAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) / Math.PI * 180.0;
                                }
                            } else {
                                parallelAngle = 90.0;
                            }
                            servoTest1.setPosition(servo1zero + degree2servo * (parallelAngle ));
                            servoTest2.setPosition(servo2zero + degree2servo * (parallelAngle ));
                            servoTest3.setPosition(servo3zero + degree2servo * (parallelAngle ));
                            servoTest4.setPosition(servo4zero + degree2servo * (parallelAngle ));
                            tgt1speed = -gamepad1.right_stick_y;
                            tgt1speed += 2800.0 * tgt1speed;
                            ((DcMotorEx) motorTest1).setVelocity(tgt1speed);
                            ((DcMotorEx) motorTest2).setVelocity(tgt1speed);
                            motorTest3.setDirection(DcMotorSimple.Direction.REVERSE);
                            motorTest4.setDirection(DcMotorSimple.Direction.REVERSE);
                            ((DcMotorEx) motorTest3).setVelocity(tgt1speed);
                            ((DcMotorEx) motorTest4).setVelocity(tgt1speed);
                            telemetry.addData("x", gamepad1.left_stick_x);
                            telemetry.addData("y", gamepad1.left_stick_y);
                            telemetry.addData("parallel angle", parallelAngle);
                            telemetry.addData("Parallel", "ON");
                            if (gamepad1.a) {
                                parallelOn = 0;
                                telemetry.addData("Parallel", "OFF");
                            }
                        }
                    } else if (gamepad1.x || steeringOn != 0) {
                        // steering
                        telemetry.addData("steering", "ON");
                        telemetry.update();
                        steeringOn = 1;
                        motorTest1.setDirection(DcMotorSimple.Direction.FORWARD);
                        motorTest2.setDirection(DcMotorSimple.Direction.FORWARD);
                        motorTest3.setDirection(DcMotorSimple.Direction.REVERSE);
                        if (steeringOn != 0) {
                            if (Math.abs(gamepad1.left_stick_x) < Math.abs(steeringDeadband)) {
                                servoTest1.setPosition(servo1zero + oneeighty / 2.0);
                                servoTest2.setPosition(servo2zero + oneeighty / 2.0);
                                servoTest3.setPosition(servo3zero + oneeighty / 2.0);
                                servoTest4.setPosition(servo3zero + oneeighty / 2.0);
                                tgt1speed = -gamepad1.right_stick_y;
                                tgt1speed += 2800 * tgt1speed;
                                ((DcMotorEx) motorTest1).setVelocity(tgt1speed);
                                ((DcMotorEx) motorTest2).setVelocity(tgt1speed);
                                ((DcMotorEx) motorTest3).setVelocity(tgt1speed);
                                ((DcMotorEx) motorTest4).setVelocity(tgt1speed);
                            } else if (gamepad1.left_stick_x > steeringDeadband) {
                                turnRadius = deltaX / gamepad1.left_stick_x;
                                alphaRight = Math.atan((turnRadius - deltaX) / deltaY) / Math.PI * 180.0;
                                alphaLeft = Math.atan((turnRadius + deltaX) / deltaY) / Math.PI * 180.0;
                                alpha1 = alphaRight;
                                alpha2 = 180.0 - alphaRight;
                                alpha3 = alphaLeft;
                                alpha4 = 180.0 - alphaLeft;
                                servoTest1.setPosition(servo1zero + degree2servo * alpha1);
                                servoTest2.setPosition(servo2zero + degree2servo * alpha2);
                                servoTest3.setPosition(servo3zero + degree2servo * alpha3);
                                servoTest4.setPosition(servo4zero + degree2servo * alpha4);
                                tgt1speed = -gamepad1.right_stick_y;
                                radiusFactor = Math.sqrt((turnRadius - deltaX) * (turnRadius - deltaX) + deltaY * deltaY) / Math.sqrt((turnRadius + deltaX) * (turnRadius + deltaX) + deltaY * deltaY);
                                tgt1speed += 2800.0 * tgt1speed;
                                ((DcMotorEx) motorTest1).setVelocity(tgt1speed * radiusFactor);
                                ((DcMotorEx) motorTest2).setVelocity(tgt1speed * radiusFactor);
                                ((DcMotorEx) motorTest3).setVelocity(tgt1speed);
                                ((DcMotorEx) motorTest4).setVelocity(tgt1speed);
                            } else if (gamepad1.left_stick_x < -1 * steeringDeadband) {
                                turnRadius = deltaX / Math.abs(gamepad1.left_stick_x);
                                alphaRight = Math.atan((turnRadius + deltaX) / deltaY) / Math.PI * 180.0;
                                alphaLeft = Math.atan((turnRadius - deltaX) / deltaY) / Math.PI * 180.0;
                                alpha2 = alphaRight;
                                alpha1 = 180.0 - alphaRight;
                                alpha4 = alphaLeft;
                                alpha3 = 180.0 - alphaLeft;
                                servoTest1.setPosition(servo1zero + degree2servo * alpha1);
                                servoTest2.setPosition(servo2zero + degree2servo * alpha2);
                                servoTest3.setPosition(servo3zero + degree2servo * alpha3);
                                servoTest4.setPosition(servo4zero + degree2servo * alpha4);
                                tgt1speed = -gamepad1.right_stick_y;
                                tgt1speed += 2800.0 * tgt1speed;
                                radiusFactor = Math.sqrt((turnRadius - deltaX) * (turnRadius - deltaX) + deltaY * deltaY) / Math.sqrt((turnRadius + deltaX) * (turnRadius + deltaX) + deltaY * deltaY);
                                ((DcMotorEx) motorTest1).setVelocity(tgt1speed);
                                ((DcMotorEx) motorTest2).setVelocity(tgt1speed);
                                ((DcMotorEx) motorTest3).setVelocity(tgt1speed * radiusFactor);
                                ((DcMotorEx) motorTest4).setVelocity(tgt1speed * radiusFactor);
                            }
                            telemetry.addData("right", alphaRight);
                            telemetry.addData("left", alphaLeft);
                            telemetry.addData("radius factor", radiusFactor);
                            telemetry.update();
                            if (gamepad1.a) {
                                steeringOn = 0;
                                telemetry.addData("steering", "OFF");
                            }
                        }
                        motorTest4.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else if (gamepad1.b) {
                        // spin around middle
                        motorTest3.setDirection(DcMotorSimple.Direction.FORWARD);
                        motorTest4.setDirection(DcMotorSimple.Direction.FORWARD);
                        spinOn = 1;
                        telemetry.addData("Spin", "ON");
                        telemetry.addData( "oneforty", oneforty );
                        telemetry.update();
                        servoTest1.setPosition(servo1zero + oneforty);
                        servoTest2.setPosition(servo2zero + (onethirty - ninety));
                        servoTest3.setPosition(servo3zero + (onethirty - ninety));
                        servoTest4.setPosition(servo4zero + oneforty);
                        while (spinOn != 0) {
                            tgt1speed = -gamepad1.right_stick_x;
                            tgt1speed += 2800 * tgt1speed;
                            ((DcMotorEx) motorTest1).setVelocity(tgt1speed);
                            ((DcMotorEx) motorTest2).setVelocity(tgt1speed);
                            ((DcMotorEx) motorTest3).setVelocity(tgt1speed);
                            ((DcMotorEx) motorTest4).setVelocity(tgt1speed);
                            if (gamepad1.a) {
                                spinOn = 0;
                            }
                        }
                        telemetry.addData("Spin", "OFF");
                        telemetry.update();
                    }
                    telemetry.update();
                }
            }
        }
    }


