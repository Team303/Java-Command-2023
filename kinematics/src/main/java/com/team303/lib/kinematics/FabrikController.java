package com.team303.lib.kinematics;

import au.edu.federation.caliko.FabrikStructure2D;
import edu.wpi.first.math.geometry.Translation3d;
import au.edu.federation.utils.Vec2f;
import au.edu.federation.caliko.FabrikChain2D;

import java.util.List;
import java.util.ArrayList;


public class FabrikController {

    FabrikStructure2D structure = new FabrikStructure2D();
    List<ArmChain> chains = new ArrayList<ArmChain>();

    public FabrikController(ArmChain... chain) {
        for (int i = 0; i < chain.length; i++) {
            structure.addChain(chain[i].chain);
            chains.add(chain[i]);
        }
    }

    public void solveNewTarget(Translation3d translation) {
        structure.solveForTarget(new Vec2f((float) translation.getX(), (float) translation.getZ()));
    }

    public List<Double> getAnglesRadians(int index) {
        FabrikChain2D chain = structure.getChain(index);

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
}
