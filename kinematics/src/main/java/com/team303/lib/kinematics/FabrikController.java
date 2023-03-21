package com.team303.lib.kinematics;

import java.util.ArrayList;
import java.util.List;

import au.edu.federation.caliko.FabrikBone2D;
import au.edu.federation.caliko.FabrikChain2D;
import au.edu.federation.caliko.FabrikChain2D.BaseboneConstraintType2D;
import au.edu.federation.caliko.FabrikJoint2D.ConstraintCoordinateSystem;
import au.edu.federation.caliko.FabrikStructure2D;
import au.edu.federation.utils.Vec2f;
import edu.wpi.first.math.geometry.Translation3d;

public class FabrikController {

    FabrikStructure2D structure = new FabrikStructure2D();
    FabrikChain2D chain = new FabrikChain2D();
    FabrikBone2D gripper;

    List<Float> segmentRatio = new ArrayList<Float>();
    List<Float> segmentLength = new ArrayList<Float>();
    List<Float[]> segmentAngleConstraint = new ArrayList<>();
    List<Vec2f> segmentInitialDirection = new ArrayList<>();

    float armLength;

    public void setArmLength(float lengthInches) {
        armLength = lengthInches;
    }

    public float getArmLength() {
        return armLength;
    }

    public void setSegmentLengthRatio(int segmentIndex, float ratio) {
        segmentRatio.add(segmentIndex, ratio);
    }

    public List<Float> getSegmentLengthRatios() {
        return segmentRatio;
    }

    public float getSegmentLengthRatio(int segmentIndex) {
        return segmentRatio.get(segmentIndex);
    }

    public void setSegmentLengths() {

        for (int i = 0; i < segmentRatio.size(); i++) {
            segmentLength.add(i, segmentRatio.get(i) * armLength);
        }
        float lengthSum = 0.0f;
        for (int i = 0; i < segmentLength.size(); i++) {
            lengthSum += segmentLength.get(i);
        }
        if (lengthSum != armLength) {
            throw new RuntimeException("Invalid lengths: Segment lengths do not add up to arm length");
        }
    }

    public List<Float> getSegmentLengths() {
        return segmentLength;
    }

    public float getSegmentLength(int segmentIndex) {
        return segmentLength.get(segmentIndex);
    }

    public void setAngleConstraint(int segmentIndex, float clockwiseConstraint, float counterclockwiseConstraint) {
        Float[] angleConstraints = new Float[] { clockwiseConstraint, counterclockwiseConstraint };
        segmentAngleConstraint.add(segmentIndex, angleConstraints);
    }

    public Float[] getAngleConstraints(int segmentIndex) {
        return segmentAngleConstraint.get(segmentIndex);
    }

    public void setSegmentInitialDirection(int segmentIndex, double angleRadians) {
        Vec2f output = new Vec2f();
        output.x = (float) Math.cos(angleRadians);
        output.y = (float) Math.sin(angleRadians);
        segmentInitialDirection.add(segmentIndex, output);
    }

    /**
     * Probably inaccurate, will fix eventually
     * 
     * @param segmentIndex
     * @return Returns the starting direction of the segment in radians
     */
    public double getSegmentInitialDirectionRadians(int segmentIndex) {
        return Math.atan2(segmentInitialDirection.get(segmentIndex).y, segmentInitialDirection.get(segmentIndex).x);
    }
    
    /**
     * Adds a bone that is globally constrained.
     * IMPORTANT: Must be called after initializeArm()
     * @param constraintAngleRadians The angle that the bone is globally constrained to
     * @param lengthInches Length of bone
     * @param clockwiseConstraintDegrees Clockwise tolerance
     * @param counterclockwiseConstraintDegrees Counterclockwise tolerance
     **/
    public void addGloballyConstrainedGripper (float constraintAngleRadians, float lengthInches, float clockwiseConstraintDegrees, float counterclockwiseConstraintDegrees ) {
        Vec2f output = new Vec2f();
        output.x = (float) Math.cos(constraintAngleRadians);
        output.y = (float) Math.sin(constraintAngleRadians);
        FabrikBone2D gripper = new FabrikBone2D(new Vec2f(1.0f, 1.0f), output, lengthInches, clockwiseConstraintDegrees, counterclockwiseConstraintDegrees);
        gripper.setJointConstraintCoordinateSystem(ConstraintCoordinateSystem.GLOBAL);
        gripper.setGlobalConstraintUV(output);
        chain.addConsecutiveBone(gripper);
    }

    public void addGloballyConstrainedGripper (float constraintAngleRadians, float lengthInches) {
        FabrikBone2D gripper = new FabrikBone2D(new Vec2f(50.0f, 50.0f), new Vec2f(1.0f,0.0f), lengthInches, 0.0f, 0.0f);
        gripper.setJointConstraintCoordinateSystem(ConstraintCoordinateSystem.GLOBAL);
        gripper.setGlobalConstraintUV(new Vec2f(1.0f,0.0f));
        chain.addConsecutiveBone(gripper);
    }
    
    /**
     * Sets a new global angle constraint
     * @param angleRadians The new angle in radians for the angle to be constrained to
     */
    public void setGripperGlobalConstraint(float angleRadians) {
        Vec2f output = new Vec2f();
        output.x = (float) Math.cos(angleRadians);
        output.y = (float) Math.sin(angleRadians);
        gripper.setGlobalConstraintUV(output);
    }

    public void initializeArm() {
        Vec2f baseEndLoc = segmentInitialDirection.get(0).times(segmentLength.get(0));
        FabrikBone2D base = new FabrikBone2D(new Vec2f(), baseEndLoc);
        base.setClockwiseConstraintDegs(segmentAngleConstraint.get(0)[0]);
        base.setAnticlockwiseConstraintDegs(segmentAngleConstraint.get(0)[1]);
        chain.addBone(base);
        chain.setBaseboneConstraintType(BaseboneConstraintType2D.GLOBAL_ABSOLUTE);
        chain.setBaseboneConstraintUV(segmentInitialDirection.get(0));
        chain.setEmbeddedTargetMode(true);
        for (int i = 1; i < segmentLength.size(); i++) {
            chain.addConsecutiveConstrainedBone(segmentInitialDirection.get(i), segmentLength.get(i),
                    segmentAngleConstraint.get(i)[0], segmentAngleConstraint.get(i)[1]);
        }
        structure.addChain(chain);
    }

    public void setMaxIterationAttempts(int maxIterations) {
        chain.setMaxIterationAttempts(maxIterations);
    }

    public void getMaxIterationAttempts() {
        chain.getMaxIterationAttempts();
    }

    public void setSolveDistanceThreshold(float toleranceInches) {
        chain.setSolveDistanceThreshold(toleranceInches);
    }

    public void getSolveDistanceThreshold() {
        chain.getSolveDistanceThreshold();
    }

    public int getNumBones() {
        return chain.getNumBones();
    }

    public void updateEmbedded(float x, float y) {
        chain.updateEmbeddedTarget(x, y);
    }

    public void updateEmbedded(Translation3d target) {
        chain.updateEmbeddedTarget((float) target.getX(), (float) target.getX());
    }

    public void solveForEmbedded() {
        chain.solveForEmbeddedTarget();
    }

    public void solveTargetIK(Translation3d target) {
        chain.solveForTarget((float) target.getX(), (float) target.getZ());
    }

    public void solveTargetIK(float xPosition, float yPosition) {
        chain.solveForTarget(xPosition, yPosition);
    }

    public List<Float> getEffectorPoint() {
        List<Float> effectorCoordinates = new ArrayList<Float>();
        effectorCoordinates.add(0, chain.getEffectorLocation().x);
        effectorCoordinates.add(1, chain.getEffectorLocation().y);
        return effectorCoordinates;
    }

    /**
     * 
     * @return joint angles in radians
     */
    public List<Double> getIKAnglesRadians() {
    Vec2f baseVectorDirection;
    double baseRadianDirection;
    List<Double> outputRadianAngles = new ArrayList<Double>();
    baseVectorDirection = chain.getBone(0).getDirectionUV().minus(new
    Vec2f((float)Math.cos(Math.PI/2),(float)Math.sin(Math.PI/2)));
    baseRadianDirection = Math.atan2(baseVectorDirection.y,
    baseVectorDirection.x);
    if (baseRadianDirection < -Math.PI / 2) {
    baseRadianDirection += Math.PI;
    }

    List<Float> location = getEffectorPoint();

    outputRadianAngles.add(-baseRadianDirection);
    for (int i = 1; i < chain.getNumBones(); i++) {
    Vec2f rotatedAngle =
    chain.getBone(i).getDirectionUV().rotateRads((float)-Math.atan2(chain.getBone(i-1).getDirectionUV().y,chain.getBone(i-1).getDirectionUV().x));
    outputRadianAngles
    .add(-Math.atan2(rotatedAngle.y,rotatedAngle.x));
    }
    if (outputRadianAngles.get(0)==-Math.PI) {
        outputRadianAngles.set(0,0.0);
       }
    return outputRadianAngles;
    }

    public List<Double> getIKAnglesDegrees() {
        List<Double> outputDegreeAngles = new ArrayList<Double>();
        List<Double> outputRadianAngles = getIKAnglesRadians();
        for (Double angle : outputRadianAngles) {
            outputDegreeAngles.add(Math.toDegrees(angle));
        }
        return outputDegreeAngles;
    }

    // Returns inches
    public float getIKPositionError() {
        Vec2f forwardKinematics = new Vec2f();
        for (int i = 0; i < chain.getNumBones(); i++) {
            Vec2f vectorDirection = chain.getBone(i).getDirectionUV();
            forwardKinematics = forwardKinematics.plus(vectorDirection.times(chain.getBone(i).length()));
        }
        Vec2f inverseKinematics = chain.getEffectorLocation();
        return Vec2f.distanceBetween(forwardKinematics, inverseKinematics);
    }
}