package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
@Config
@TeleOp
public class Teleop extends OpMode {
    Drivebase drivebase;
    Transfer transfer;
    Shooter shooter;
    Intake intake;
    PinpointLocalizer localizer;
    private double velocityPTarget = 1100;
    private GoBildaPinpointDriver pinpoint;
    FtcDashboard dash;
    private static Telemetry myTe;
    private static int alliance =1 ;

    @Override
    public void init(){
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        dash = FtcDashboard.getInstance();
        myTe = new MultipleTelemetry(dash.getTelemetry(), telemetry);
        intake = new Intake(hardwareMap);
        drivebase = new Drivebase(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        myTe.update();
    }

    @Override
    public void loop(){
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        drivebase.drive(drive, strafe, turn);
        shooting_machine();
        myTe.addData("Pollen Velocity", shooter.getVelocityP());
        myTe.addData("Nectar Velocity", shooter.getVelocityN());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        localizer.update();
        myTe.addData("X", localizer.pose().x());
        myTe.addData("Y", localizer.pose().y());
        pinpoint.update();
        double heading = pinpoint.getPosition().getHeading(AngleUnit.DEGREES);
        if(gamepad1.xWasPressed()){
            alliance = alliance==1 ? 1 : 0;
        }
        myTe.update();
        telemetry.update();
    }
    public void shooting_machine() {
        if (gamepad1.left_trigger > 0.3||gamepad2.left_trigger > 0.3) {
            shooter.setPower(-1.0);
            intake.setPower(1.0);
            transfer.setPower(1.0);
        }
        else {
            if (shooter.getVelocityP() < (velocityPTarget * 0.98)) {
                shooter.setPower(1.0);
            } else {
                shooter.setVelocityP(velocityPTarget);
            }
            if (gamepad1.right_trigger > 0.3||gamepad2.right_trigger>0.3) {
                if (shooter.getVelocityP() >= (velocityPTarget * 0.98)) {
                    boolean go = true;
                    while(go){
                        go = drivebase.turnToAprilTag(alliance);
                        if(gamepad1.right_trigger<.3){
                            go = false;
                        }
                    }
                    transfer.setPower(-1.0);
                } else {
                    transfer.setPower(0);
                }
            }
            else if (gamepad1.right_bumper||gamepad2.right_bumper) {
                intake.setPower(-1.0);
                transfer.setPower(0);
            }
            else {
                intake.setPower(0);
                transfer.setPower(0);
            }
        }
    }
}