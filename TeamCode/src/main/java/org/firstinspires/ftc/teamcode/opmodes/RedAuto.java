package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.BarcodeLocation;

import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_LOW;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_MIDDLE;


@Autonomous(name = "Red Auto", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedAuto extends AbstractAuto {

    @Override
    public void setAlliance() {
        this.alliance = RED;
    }

    @Override
    public void initializeSteps(BarcodeLocation location) {
        addMovement(0, 4, 0.5);
        addTurnAbsolute(45);
        switch(location) {
            case LEFT:
                addSlide(SLIDE_DROP_LOW);
                break;
            case MIDDLE:
                addSlide(SLIDE_DROP_MIDDLE);
                break;
            case RIGHT:
                addSlide(SLIDE_DROP_HIGH);
                break;
            case UNKNOWN:
                break;

        }
        addDelay(22);

    }
}