package org.firstinspires.ftc.teamcode.vikingroboticsteamcode2020.Fall2020.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;
import java.util.ArrayList;

interface Hold {
    void begin();
    boolean until();
    void end();
}

class Step extends LinearOpMode {
    protected List<Hold> holds;
    Step(List<Hold> holds) {
        this.holds = holds;
    }
    public Step clone() {
        return new Step(new ArrayList<>(this.holds));
    }
    public void runOpMode() {
        Step localThis = this.clone();
        for(Hold hold : localThis.holds)
            hold.begin();
        while(opModeIsActive() && !localThis.holds.isEmpty()) {
            for(Hold hold : localThis.holds) {
                if(hold.until()) {
                    hold.end();
                    localThis.holds.remove(hold);
                }
            }
            sleep(10);
        }
    }
}

public class Procedure extends LinearOpMode {
    List<Step> steps;
    Procedure(List<Step> steps) {
        this.steps = steps;
    }
    public Procedure clone() {
        return new Procedure(new ArrayList<>(this.steps));
    }
    public void runOpMode() {
        for(Step step : this.steps) {
            step.runOpMode();
            sleep(10);
        }
    }
}

