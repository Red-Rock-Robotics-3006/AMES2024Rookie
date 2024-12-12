package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final double topPosition;
    private final double bottomPosition;
    private double targetPosition;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kF = 0;


    private CANSparkMax elevatorMotor = new CANSparkMax(45, CANSparkMax.MotorType.kBrushless);
    private PIDController controller = new PIDController(kP, kI, kD);
    public Elevator(){
        super("Elevator");
        this.elevatorMotor.restoreFactoryDefaults();
        this.elevatorMotor.setInverted(false);
        this.elevatorMotor.setIdleMode(CANSparkMax.IdleMode.kCoast); 

        this.controller.setTolerance(0.02); // TODO FILLER

        this.bottomPosition = this.elevatorMotor.getEncoder().getPosition();
        this.topPosition = this.bottomPosition + 32.75;
        this.targetPosition = this.bottomPosition;

        SmartDashboard.putNumber("elevator/kP", this.kP);
        SmartDashboard.putNumber("elevator/kD", this.kD);
        SmartDashboard.putNumber("elevator/kF", this.kF);
        SmartDashboard.putNumber("elevator/postion", this.elevatorMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator/target", this.targetPosition);
    }
    private void setSpeed(double speed){
        this.elevatorMotor.set(speed);
    }
 
    private void goTop(){
        this.targetPosition = this.topPosition;
    }
    private void goBottom(){
        this.targetPosition = this.bottomPosition;
    }

    public Command topCommand(){
        return new InstantCommand(
            () -> {this.goTop();},
            this
        );
    }
    public Command bottomCommand(){
        return new InstantCommand(
            () -> {this.goBottom();},
            this
        );
    }

    @Override
    public void periodic(){
        
        
        this.kP = SmartDashboard.getNumber("elevator/kP", this.kP);
        this.kD = SmartDashboard.getNumber("elevator/kD", this.kD);
        this.kF = SmartDashboard.getNumber("elevator/kF", this.kF);
        
        this.controller.setP(this.kP);
        this.controller.setD(this.kD);

        double currentPosition = elevatorMotor.getEncoder().getPosition();
        this.setSpeed(this.kF + controller.calculate(currentPosition, this.targetPosition));
        
        SmartDashboard.putNumber("elevator/kP", this.kP);
        SmartDashboard.putNumber("elevator/kD", this.kD);
        SmartDashboard.putNumber("elevator/kF", this.kF);
        SmartDashboard.putNumber("elevator/postion", currentPosition);
        SmartDashboard.putNumber("elevator/target", this.targetPosition);
    }
    
}
