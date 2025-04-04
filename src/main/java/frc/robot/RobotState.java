package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotState {
    private static RobotState robotState;

    public static class IntakeState {
        public static boolean isEmpty = true;
       

        public static void setIsEmpty(boolean isEmpty) {
            IntakeState.isEmpty = isEmpty;
        }

    }

    public static class SwerveState{
        public static Command preciseCommand = Commands.none();

        public static void setCommand(Command comm){
            preciseCommand = comm;
        }

        public static Command getCommand(){
            return preciseCommand;
        }
    }

    public static class CoreState {

        public static boolean isTeleop() {
            return edu.wpi.first.wpilibj.RobotState.isTeleop();
        }

        public static boolean isAutonomous() {
            return edu.wpi.first.wpilibj.RobotState.isAutonomous();
        }

        public static boolean isDisabled() {
            return edu.wpi.first.wpilibj.RobotState.isDisabled();
        }

    }

    // public static class SwerveState {
    //     public static boolean swerveSlowMode;

    //     public static void setSwerveSlowMode(boolean mode){
    //         swerveSlowMode = mode;
    //     }
    // }

    public static RobotState getInstance() {
        if (robotState == null) {
            robotState = new RobotState();
        }
        return robotState;
    }
    
}
