// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;

/** Add your docs here. */
public class TrajectoryPlanner4920 {

    public TrajectoryPlanner4920(){

    }

    public enum MechState{
        CoralStation,
        SafeTransition,
        Level1,
        Level2,
        Level3,
        Level4,
        AlgaeLow,
        AlgaeHigh,
        TransportHigh,
        TransportLow,
        StartingConfig,
        Holding
    }

    public static class Node{
        private MechState name;
        private double wristGoal;
        private double elbowGoal;
        private double elevatorGoal;

        public Node(MechState name, double wrist, double elbow, double elevator){
            this.name = name;
            this.wristGoal = wrist;
            this.elbowGoal = elbow;
            this.elevatorGoal = elevator;
        }

        public Node(){
            this.name = null;
            this.wristGoal = 0;
            this.elbowGoal = 0;
            this.elevatorGoal = 0;
        }

        public Node(Node n){
            this.name = n.name;
            this.wristGoal = n.wristGoal;
            this.elbowGoal = n.elbowGoal;
            this.elevatorGoal = n.elevatorGoal;
        }

        public Node(MechState name, DoubleSupplier wrist, DoubleSupplier elbow, DoubleSupplier elevator){
            this.name = name;
            this.wristGoal = wrist.getAsDouble();
            this.elbowGoal = elbow.getAsDouble();
            this.elevatorGoal = elevator.getAsDouble();
        }

        public MechState getName(){
            return name;
        }

        public double getWristGoal(){
            return wristGoal;
        }

        public double getElbowGoal(){
            return elbowGoal;
        }

        public double getElevatorGoal(){
            return elevatorGoal;
        }

        public double getDistance(Node n){
            Translation3d cur = new Translation3d(this.getElbowGoal(), this.getElevatorGoal(), this.getWristGoal());
            return cur.getDistance(new Translation3d(n.getElbowGoal(), n.getElevatorGoal(),n.getWristGoal()));
        }
    }

    public static MechState getClosestNode(HashMap<MechState,Node> list, Node currentState){
        Node res = currentState;
        double minDistance = Double.MAX_VALUE;
        for (Node n : list.values()){
            if (currentState.getDistance(n) < minDistance){
                minDistance = currentState.getDistance((n));
                res = n;
            }
        }
        return res.getName();
    }
}
