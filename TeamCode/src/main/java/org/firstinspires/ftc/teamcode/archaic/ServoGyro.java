package org.firstinspires.ftc.teamcode.archaic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.called.HWRobot;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Autonomous made to test gyro and servos
 * Made By Gaurav
 */

@TeleOp(name="ServoGyro", group="Testing")
//@Disabled
public class ServoGyro extends LinearOpMode {
    // Declare OpMode members.
    //private ElapsedTime runtime = new ElapsedTime();
    HWRobot robot = new HWRobot();

    // State used for updating telemetry
    Orientation angles;


    //double roll = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);
    //double pitch = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle);


    @Override
    public void runOpMode() {
        robot.getOpModeData(telemetry,hardwareMap);robot.init("imu");
        // Set up our telemetry dashboard
        //composeTelemetry();

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        //imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // Loop and update the dashboard
        while (opModeIsActive()) {
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
            telemetry.addData("heading" , heading);
            //telemetry.addData("roll" , roll);
            //telemetry.addData("pitch" , pitch);
            telemetry.update();

        }
    }
}
