package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Door extends SubsystemBase{
    private final int DOOR_MOTOR_ID = 51; 
    private final CANSparkMax m_doorMotor = new CANSparkMax(DOOR_MOTOR_ID, CANSparkMax.MotorType.kBrushless);
    
    public double kP = 1.0; //TODO
    private double kI = 0.0; 
    private double kD = 0.0; //TODO
    public double kF = 0.0; //TODO
    private PIDController controller = new PIDController(kP, kI, kD);
    
    private double openPosition; 
    private double closedPosition;
    private double targetPosition;

    public Door(Elevator elevator){
        super("Door");
        this.m_doorMotor.restoreFactoryDefaults();
        this.m_doorMotor.setInverted(false);
        this.m_doorMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        SmartDashboard.putNumber("door/kF", kF);
        SmartDashboard.putNumber("door/kP", kP);
        SmartDashboard.putNumber("door/kD", kD);
        SmartDashboard.putNumber("door/position", m_doorMotor.getEncoder().getPosition());
        this.closedPosition = this.m_doorMotor.getEncoder().getPosition();
        this.openPosition = this.closedPosition + 7.65;
        this.targetPosition = this.closedPosition;
    }
    
    public void set(double motorSpeed){
        this.m_doorMotor.set(motorSpeed);
    }
    public void openDoor(){
        this.targetPosition = this.openPosition;
    }
    public void closeDoor(){
        this.targetPosition = this.closedPosition; 
    }
    public void doorTiltSixty(){
        this.targetPosition = this.closedPosition + 2.125;
    }
    @Override
    public void periodic(){
        double feedfoward = Math.sin(this.m_doorMotor.getEncoder().getPosition()/ this.openPosition * Math.PI/2) * kF;
        this.set(0.05*(feedfoward + this.controller.calculate(this.m_doorMotor.getEncoder().getPosition(), this.targetPosition)));
        
        this.kF = SmartDashboard.getNumber("door/kF", kF);
        this.kD = SmartDashboard.getNumber("door/kD", kD);
        this.kP = SmartDashboard.getNumber("door/kP", kP);
        
        this.targetPosition = SmartDashboard.getNumber("door/target", targetPosition);
        
        
        SmartDashboard.putNumber("door/target", targetPosition);
        SmartDashboard.putNumber("door/kF", kF);
        SmartDashboard.putNumber("door/kP", kP);
        SmartDashboard.putNumber("door/kD", kD);
        SmartDashboard.putNumber("door/position", m_doorMotor.getEncoder().getPosition());
        
        this.controller.setP(kP);
        this.controller.setD(kD);
    }
    
}


