package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;

public class Intake {
    private Alert motor1disconnectedAlert;
    private Alert motor2disconnectedAlert;
    private IntakeIO intakeIO;


    public Intake(IntakeIO io) {
        this.intakeIO = io;

        motor1disconnectedAlert = new Alert("intake motor disconnected!", null);
        motor2disconnectedAlert = new Alert("intake motor disconnected!", null);

    }
    public void periodic(){
        intakeIO.updateInputs(null);
    }
    
}

