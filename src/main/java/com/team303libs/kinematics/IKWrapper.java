package com.team303libs.kinematics;

import java.util.HashMap;

import au.edu.federation.caliko.FabrikBone2D;
import au.edu.federation.caliko.FabrikChain2D;
import au.edu.federation.caliko.FabrikChain2D.BaseboneConstraintType2D;
import au.edu.federation.caliko.FabrikStructure2D;
import au.edu.federation.utils.Vec2f;
import edu.wpi.first.math.geometry.Translation3d;
//import com.team303.robot.Robot;

public class IKWrapper {

    FabrikStructure2D structure = new FabrikStructure2D();
    FabrikChain2D chain = new FabrikChain2D();

    HashMap<Integer, Float> segmentRatio = new HashMap<>();
    HashMap<Integer, Float> segmentLength = new HashMap<>();
    HashMap<Integer, Float[]> segmentAngleConstraint = new HashMap<>();
    HashMap<Integer, Vec2f> segmentInitialDirection = new HashMap<>();

    float armLength;

    public void setArmLength(float lengthInches) {
        armLength = lengthInches;
    }
    public float getArmLength() {
        return armLength;
    }
    public void setSegmentLengthRatio(int segmentIndex, float ratio) {
        segmentRatio.put(segmentIndex, ratio);
    }
    public float[] getSegmentLengthRatios() {
        float[] output = new float[segmentRatio.size()];
        int x=0;
        for (float i : segmentRatio.values()) {
            output[x] = i;
            x++;
        }
        return output;
    }
    public float getSegmentLengthRatio(int segmentIndex) {
        return segmentRatio.get(segmentIndex);
    }
    public void setSegmentLengths() {
        int x=0;
        for (float i : segmentRatio.values()) {
            segmentLength.put(x,i*armLength);
            x++;
        }
    }
    public float[] getSegmentLengths() {
        float[] output = new float[segmentLength.size()];
        int x=0;
        for (float i : segmentLength.values()) {
            output[x] = i;
            x++;
        }
        return output;
    }
    public float getSegmentLength(int segmentIndex) {
        return segmentLength.get(segmentIndex);
    }
    public void setAngleConstraint(int segmentIndex, float clockwiseConstraint, float counterclockwiseConstraint) {
        Float[] angleConstraints = new Float[]{clockwiseConstraint,counterclockwiseConstraint};
        segmentAngleConstraint.put(segmentIndex,angleConstraints);
    }
    public Float[] getAngleConstraints(int segmentIndex) {
        return segmentAngleConstraint.get(segmentIndex);          
    }
    public void setSegmentInitialDirection(int segmentIndex,double angleRadians) {
        Vec2f output = new Vec2f();
        output.x = (float)Math.cos(angleRadians);
        output.y = (float)Math.sin(angleRadians);
        segmentInitialDirection.put(segmentIndex,output);
    }
    //Returns radians
    public double getSegmentInitialDirectionRadians(int segmentIndex, Vec2f initialDirection) {
        return Math.atan2(initialDirection.y,initialDirection.x);
    }
    public void initializeArm() {
        Vec2f baseEndLoc = segmentInitialDirection.get(0).times(segmentLength.get(0));
        FabrikBone2D base = new FabrikBone2D(new Vec2f(),baseEndLoc);
        base.setClockwiseConstraintDegs(segmentAngleConstraint.get(0)[0]);
        base.setAnticlockwiseConstraintDegs(segmentAngleConstraint.get(0)[1]);
        chain.addBone(base);
        chain.setBaseboneConstraintType(BaseboneConstraintType2D.GLOBAL_ABSOLUTE);
        chain.setBaseboneConstraintUV(segmentInitialDirection.get(0));
        for(int i=0;i<segmentLength.size();i++) {
            chain.addConsecutiveConstrainedBone(segmentInitialDirection.get(i),segmentLength.get(i),segmentAngleConstraint.get(i)[0],segmentAngleConstraint.get(i)[1]);
        }
        structure.addChain(chain);
    }
    public void solveTargetIK(Translation3d target) {
        chain.solveForTarget((float)target.getX(),(float)target.getZ());
    }
    //Returns radians
    public double[] getIKAnglesRadians() {
        double[] outputAnglesRadians = new double[chain.getNumBones()];
        Vec2f vectorDirection = new Vec2f();
        for (int i=0;i<chain.getNumBones(); i++) {
            vectorDirection = chain.getBone(i).getDirectionUV();
            outputAnglesRadians[i] = Math.atan2(vectorDirection.y,vectorDirection.x);
        }
        return outputAnglesRadians;
    }

    public float getIKPositionError() {
        Vec2f forwardKinematics = new Vec2f();
        for (int i=0;i<chain.getNumBones(); i++) {
           Vec2f vectorDirection = chain.getBone(i).getDirectionUV();
           forwardKinematics = forwardKinematics.plus(vectorDirection.times(chain.getBone(i).length()));
        }
        Vec2f inverseKinematics = chain.getEffectorLocation();
        return Vec2f.distanceBetween(forwardKinematics,inverseKinematics);
    }
}
