package com.redstorm509.stormkit.math;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/**
 * A simple 2-dimensional inverse kinematics solver.
 * Reference code: https://github.com/FRC-509/inverse-kinematics/blob/master/src/kinematics.rs
 */
public class IKSolver {

    public static class SegmentParams {
        public double pivotMinExtentRadians;
        public double pivotMaxExtentRadians;
        public double pivotRadians;
        public double lengthMeters;
        
        /**
         * @param pivotMinExtentRadians The minimum pivot angle permitted for the segment in radians.
         * @param pivotMaxExtentRadians The maximum pivot angle permitted for the segment in radians.
         * @param pivotRadians The initial angle of the segment in radians.
         * @param lengthMeters The length of the segment in meters.
         */
        public SegmentParams(double pivotMinExtentRadians, double pivotMaxExtentRadians, double pivotRadians, double lengthMeters) {
            this.pivotMinExtentRadians = pivotMinExtentRadians;
            this.pivotMaxExtentRadians = pivotMaxExtentRadians;
            this.pivotRadians = pivotRadians;
            this.lengthMeters = lengthMeters;
        }

        public void applyDelta(double deltaPivotRadians) {
            this.pivotRadians = MathUtil.clamp(this.pivotRadians + deltaPivotRadians, this.pivotMinExtentRadians, this.pivotMaxExtentRadians);
        }
    } 

    private ArrayList<SegmentParams> segments;
    private int maxIterations;

    /**
     * Constructs an IKSolver with the given segment information. Defaults to 100 maximum IK iterations.
     *
     * @param segments An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
     */
    public IKSolver(ArrayList<SegmentParams> segments) {
        this.segments = segments;
        this.maxIterations = 100;
    }

    /**
     * Constructs an IKSolver with the given segment information, and a desired maximum number of IK iterations.
     *
     * @param segments An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
     * @param maxIterations The maximum number of iterations permitted for inverse kinematics.
     */
    public IKSolver(ArrayList<SegmentParams> segments, int maxIterations) {
        this.segments = segments;
        this.maxIterations = maxIterations;
    }

    /**
     * Updates the segment information. Run this with sensor information before doing any calculations.
     *
     * @param segments An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
     */
    public void updateSegments(ArrayList<SegmentParams> segments) {
        this.segments = segments;
    }

    /**
     * Gets the current segment data. Run this after performing calculations.
     *
     * @return An ArrayList of 2-component vectors, where each vector's x-component corresponds to a joint's pivot angle in radians, and its y-component corresponds to a joint's arm length in meters. 
     */
    public ArrayList<SegmentParams> getSegments() {
        return this.segments;
    }

    /**
     * Performs forward kinematics on the arm segments, and returns an ArrayList of joint positions in meters.
     * 
     * @return An ArrayList of joint position vectors in meters.
     */
    public ArrayList<Vector<N2>> forwardKinematics() {
        ArrayList<Vector<N2>> positions = new ArrayList<>();
        double theta = 0.0;
        double x = 0.0;
        double y = 0;

        for (SegmentParams segment : this.segments) {
            double pivotRadians = segment.pivotRadians;
            double lengthMeters = segment.lengthMeters;
            theta += pivotRadians;
            x += Math.cos(theta) * lengthMeters;
            y += Math.sin(theta) * lengthMeters;

            positions.add(VecBuilder.fill(x, y));
        }

        return positions;
    }

    /**
     * Performs inverse kinematics on the arm segments given a desired end-effector position.
     * 
     * @param desiredPos A desired end effector position in meters.
     */
    public void inverseKinematics(Vector<N2> desiredPos) {
        for (int n = 0; n < this.maxIterations; n++) {
            for (int i = 0; i < this.segments.size(); i++) {
                ArrayList<Vector<N2>> positions = this.forwardKinematics();
                
                // position of current segment
                Vector<N2> currentSegmentPos = positions.get(i);
                // position of end effector
                Vector<N2> endEffectorPos = positions.get(positions.size() - 1);
                
                if (new Vector<N2>(desiredPos.minus(endEffectorPos)).norm() < 0.01) {
                    return;
                }

                Vector<N2> currentSegmentToEndEffector = new Vector<N2>(endEffectorPos.minus(currentSegmentPos));
                Vector<N2> currentSegmentToDesiredPos = new Vector<N2>(desiredPos.minus(currentSegmentPos));

                // compute the angle of the triangle created between
                // the current segment point, the last segment point,
                // and the desired end position
                double a = currentSegmentToEndEffector.norm();
                double b = currentSegmentToDesiredPos.norm();
                double dot = currentSegmentToEndEffector.dot(currentSegmentToDesiredPos);
                double delta = Math.acos(dot / (a * b));

                // calculate whether delta calculates for the necessary positive
                // or negative offset to the current segment angle using its normal
                double direction = - currentSegmentToEndEffector.get(1, 0) * currentSegmentToDesiredPos.get(0, 0) + currentSegmentToEndEffector.get(0, 0) * currentSegmentToDesiredPos.get(1, 0);
                
                if (Math.abs(delta) < 0.0001) {
                    if (direction < 0) {
                        this.segments.get(i).applyDelta(-delta);
                    } else {
                        this.segments.get(i).applyDelta(delta);
                    }
                }
            }
        }
    }
}
