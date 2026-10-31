

package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.subsystems.limeLightV2;


public class Drivebase {


    private DcMotor frontLeft, frontRight;
    private DcMotor backLeft, backRight;


    private GoBildaPinpointDriver pinpoint;
    private limeLightV2 lime;


    // Tuning values
    public static double AIM_KP = 0.015;
    public static double MAX_TURN_POWER = 0.35;
    public static double AIM_TOLERANCE = 2.5;


    private double currentTX = 0;
    private boolean targetFound = false;
    private boolean aligned = false;


    public Drivebase(HardwareMap hardwareMap) {


        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);


        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);


        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class, "pinpoint");


        lime = new limeLightV2(hardwareMap);
    }


    public void drive(double drive, double strafe, double turn) {


        double denominator = Math.max(
                Math.abs(drive) +
                        Math.abs(strafe) +
                        Math.abs(turn), 1.0);


        double frontLeftPower =
                (drive + strafe + turn) / denominator;


        double frontRightPower =
                (drive - strafe - turn) / denominator;


        double backLeftPower =
                (drive - strafe + turn) / denominator;


        double backRightPower =
                (drive + strafe - turn) / denominator;


        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }


    // Returns true ONLY when a target is detected and centered.
    public boolean turnToAprilTag(int allianceType) {


        double[][] tags = lime.limeLight(allianceType);


        double closestDistance = Double.MAX_VALUE;
        double targetAngle = 0;


        targetFound = false;
        aligned = false;


        // Find closest visible alliance AprilTag
        for (double[] tag : tags) {


            if (tag == null || tag.length < 3) {
                continue;
            }


            if (tag[0] < closestDistance) {


                closestDistance = tag[0];
                targetAngle = tag[1];
                targetFound = true;
            }
        }


        // No valid AprilTag
        if (!targetFound) {
            currentTX = 0;
            stopRobot();
            return false;
        }


        currentTX = targetAngle;


        // Check if robot is centered
        if (Math.abs(targetAngle) <= AIM_TOLERANCE) {


            stopRobot();
            aligned = true;
            return true;
        }


        // Proportional controller
        double turnPower = AIM_KP * targetAngle;


        // Limit motor power
        turnPower = Math.max(
                -MAX_TURN_POWER,
                Math.min(MAX_TURN_POWER, turnPower)
        );


        turnRobot(turnPower);


        return false;
    }


    public void turnRobot(double power) {


        // Mecanum rotation using the same convention as drive()
        frontLeft.setPower(power);
        backLeft.setPower(power);


        frontRight.setPower(-power);
        backRight.setPower(-power);
    }


    public void stopRobot() {


        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    public double getTargetTX() {
        return currentTX;
    }


    public boolean hasTarget() {
        return targetFound;
    }


    public boolean isAligned() {
        return aligned;
    }
}