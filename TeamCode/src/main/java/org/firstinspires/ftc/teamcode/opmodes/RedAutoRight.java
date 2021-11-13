package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.BarcodeLocation;

import static org.firstinspires.ftc.teamcode.util.Alliance.RED;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_LOW;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_LOW_POS1;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_DROP_MIDDLE;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_MID;
import static org.firstinspires.ftc.teamcode.util.Configurables.HOPPER_START;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_HIGH;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_LOW;
import static org.firstinspires.ftc.teamcode.util.Configurables.SLIDE_DROP_MIDDLE;

@Autonomous(name = "Red Right", group = "Competition", preselectTeleOp = "Red TeleOp")
public class RedAutoRight extends AbstractAuto {

    @Override
    public void setAlliance() {
        this.alliance = RED;
    }

    @Override
    public void initializeSteps(BarcodeLocation location) {
        addHopper(0, 0.25);
        addMovement(0, -40, 0.5);
        addMovement(6, 22, 0.5);
        addTurnAbsolute(45);
        addMovement(0, -10, 0.5);
        switch(location) {
            case LEFT:
                addSlide(SLIDE_DROP_HIGH);
                addHopper(0, HOPPER_DROP_LOW_POS1);
                addSlide(SLIDE_DROP_LOW);
                addHopper(1, HOPPER_DROP_LOW);
                addSlide(SLIDE_DROP_HIGH);
                addHopper(0, HOPPER_MID.l);
                addSlide(0);
                break;
            case MIDDLE:
                addSlide(SLIDE_DROP_HIGH);
                addHopper(1, HOPPER_DROP_HIGH);
                addHopper(0.2, HOPPER_MID.l);
                addSlide(0);
                break;
            case RIGHT:
            case UNKNOWN:
                addSlide(SLIDE_DROP_HIGH);
                addHopper(1, HOPPER_DROP_HIGH);
                addHopper(0.2, HOPPER_MID.l);
                addSlide(0);
                break;
        }
        addHopper(0, 0.2);
        addMovement(5, 5, 0.5);
        addTurnAbsolute(90);
        addMovement(0, 50, 0.5);
    }
}