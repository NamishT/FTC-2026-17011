package org.firstinspires.ftc.teamcode.leds;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.Arrays;

@Config
@TeleOp
public class ledTest extends LinearOpMode{

    public GoBildaPrismDriver prism;
    public HuskyLens huskyLens;
    public boolean isRunning;

//    Blue 1 Yellow 2 Red 3

    public void runOpMode() {
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        if (!huskyLens.knock()) {
            telemetry.addLine("HuskyLens is not working");
        } else {
            telemetry.addLine("HuskyLens is working");
        }

        telemetry.update();

        PrismAnimations.Solid firstLayer = new PrismAnimations.Solid(Color.GREEN, 100, 0, 1);
        PrismAnimations.Solid secondLayer = new PrismAnimations.Solid(Color.RED, 100, 2, 3);

        prism.clearAllAnimations();

        huskyLens.selectAlgorithm(
                HuskyLens.Algorithm.COLOR_RECOGNITION
        );


        waitForStart();
        resetRuntime();

        isRunning = true;

        while (isRunning) {

            int amount = 0;

            HuskyLens.Block[] blocks = huskyLens.blocks();

            sleep(1000);

            telemetry.update();
            switch (blocks.length) {
                case 0:
                    firstLayer = new PrismAnimations.Solid(Color.RED, 100, 0, 5);
                    secondLayer = new PrismAnimations.Solid(Color.RED, 100, 6, 11);
                    break;
                case 1:
                    firstLayer = new PrismAnimations.Solid(Color.GREEN, 100, 0, 5);
                    secondLayer = new PrismAnimations.Solid(Color.RED, 100, 6, 11);
                    break;
                case 2:
                    firstLayer = new PrismAnimations.Solid(Color.GREEN, 100, 0, 5);
                    secondLayer = new PrismAnimations.Solid(Color.GREEN, 100, 6, 11);
                    break;

//                    case 3:
//                        greenSolidAnimation = new PrismAnimations.Solid(Color.GREEN, 100, 0, 5);
//                        redAnimation = new PrismAnimations.Solid(Color.RED, 100, 6, 21);
//                        break;
//
//                    case 4:
//                        greenSolidAnimation = new PrismAnimations.Solid(Color.GREEN, 100, 0, 5);
//                        redAnimation = new PrismAnimations.Solid(Color.RED, 100, 6, 21);
//                        break;

            }

            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, firstLayer);
            prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, secondLayer);

        }

    }
}
