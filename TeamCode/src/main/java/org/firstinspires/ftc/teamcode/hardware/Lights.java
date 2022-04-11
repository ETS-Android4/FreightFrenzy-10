package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.util.Constants.LIGHTS;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Locale;

@Config
public class Lights {
    public static int NUMBER = 1;//80 solid
    public static int RED = 80;
    public static int REDFULL = 3;
    public static int REDSCORING = 5;//5
    public static int BLUE = 93;
    public static int BLUEFULL = 2;
    public static int BLUESCORING = 5;//5

    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    public Lights(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, LIGHTS);
        setPattern();
    }

    public void setPattern() {
        pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(NUMBER);
        blinkinLedDriver.setPattern(pattern);
    }

    public void setPattern(int patternNumber) {
        pattern = RevBlinkinLedDriver.BlinkinPattern.fromNumber(patternNumber);
        blinkinLedDriver.setPattern(pattern);
    }

    public void nextPattern() {
        pattern = pattern.next();
        blinkinLedDriver.setPattern(pattern);
    }

    public void previousPattern() {
        pattern = pattern.previous();
        blinkinLedDriver.setPattern(pattern);
    }

    public String getTelemetry() {
        return String.format(Locale.US, "Pattern: %s", pattern.toString());
    }
}
