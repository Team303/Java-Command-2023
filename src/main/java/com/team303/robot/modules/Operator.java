package com.team303.robot.modules;

import static com.team303.robot.Robot.heldObject;

import com.team303.lib.math.Point2D;
import com.team303.robot.Robot.HeldObject;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//TODO: Only one queue at a time, only accept queue if node is not already queued   
public class Operator extends SubsystemBase {
    public static final ShuffleboardTab OPERATOR_TAB = Shuffleboard.getTab("Operator");
    public static final NetworkTable operator = NetworkTableInstance.getDefault().getTable("Operator");
    public static final GenericEntry[][] nodes = new GenericEntry[3][9];
    public static final int[][] nodeStateValues = new int[3][9];
    public static final int[][] nodeSuperStateValues = new int[3][9];
    public Point2D hoverValue = new Point2D(0,0);
    public Point2D queuedValue;

    public static enum NodeState {
        NONE(0),
        CONE(1),
        CUBE(2),
        QUEUED(4);

        public final int value;

        private NodeState(int value) {
            this.value = value;
        }
    }

    public static enum NodeSuperState {
        NONE(0),
        HOVER(3),
        INVALID(5);

        public final int value;

        private NodeSuperState(int value) {
            this.value = value;
        }
    }

    public Operator() {
        for (int i = 0; i < nodes.length; i++) {
            for (int j = 0; j < nodes[i].length; j++) {
                nodes[i][j] = OPERATOR_TAB.add("Node r" + i + " c" + j, 0).withPosition(j + 1, i + 1)
                        .withWidget("State of Node").getEntry();
            }
        }
        nodeSuperStateValues[0][0] = NodeSuperState.HOVER.value;
    }

    public void moveDown() {
        nodeSuperStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeSuperState.NONE.value;
        for (int k = 1; k < nodeSuperStateValues.length + 1; k++) {
            int newRow = hoverValue.xAsInt() + k;
            if (newRow > 2) {
                newRow -= 3;
            }
            if (nodeSuperStateValues[newRow][hoverValue.xAsInt()] == NodeSuperState.NONE.value) {
                hoverValue.x = newRow;
                nodeSuperStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeSuperState.HOVER.value;
                return;
            }
        }
    }

    public void moveUp() {
        nodeSuperStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeSuperState.NONE.value;
        for (int k = 1; k < nodeSuperStateValues.length + 1; k++) {
            int newRow = hoverValue.xAsInt() - k;
            if (newRow < 0) {
                newRow += 3;
            }
            if (nodeSuperStateValues[newRow][hoverValue.yAsInt()] == NodeSuperState.NONE.value) {
                nodeSuperStateValues[newRow][hoverValue.yAsInt()] = NodeSuperState.HOVER.value;
                hoverValue.x = newRow;
                return;
            }
        }
    }

    public void moveLeft() {
        nodeSuperStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeSuperState.NONE.value;
        for (int k = 1; k < nodeSuperStateValues[hoverValue.xAsInt()].length + 1; k++) {
            int newCol = hoverValue.yAsInt() - k;
            if (newCol < 0) {
                newCol += 9;
            }
            if (nodeSuperStateValues[hoverValue.xAsInt()][newCol] == NodeSuperState.NONE.value) {
                nodeSuperStateValues[hoverValue.xAsInt()][newCol] = NodeSuperState.HOVER.value;
                hoverValue.y = newCol;
                return;
            }
        }
    }

    public void moveRight() {
        nodeSuperStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeSuperState.NONE.value;
        for (int k = 1; k < nodeSuperStateValues[hoverValue.xAsInt()].length + 1; k++) {
            int newCol = hoverValue.yAsInt() + k;
            if (newCol > 8) {
                newCol -= 9;
            }
            if (nodeSuperStateValues[hoverValue.xAsInt()][newCol] == NodeSuperState.NONE.value) {
                nodeSuperStateValues[hoverValue.xAsInt()][newCol] = NodeSuperState.HOVER.value;
                hoverValue.y = newCol;
                return;
            }
        }
    }

    public void setPiece() {
        if (nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] == NodeState.CUBE.value || nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] == NodeState.CONE.value) {
            nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeState.NONE.value;
        } else if (hoverValue.xAsInt()>1) {
            nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeState.CUBE.value;
        } else if (hoverValue.yAsInt() % 3 == 0 || hoverValue.yAsInt() % 3 == 2) {
            nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeState.CONE.value;
        } else
        {
            nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeState.CUBE.value;
        }
    }

    public void queuePlacement() {
        if (nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] == NodeState.QUEUED.value) {
            nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeState.NONE.value;
        } else {
            if (queuedValue == null) {
            nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()] = NodeState.QUEUED.value;
            queuedValue = new Point2D(hoverValue.x,hoverValue.y);
            } else {
            nodeStateValues[queuedValue.xAsInt()][queuedValue.yAsInt()]=NodeState.NONE.value;
            nodeStateValues[hoverValue.xAsInt()][hoverValue.yAsInt()]=NodeState.QUEUED.value;
            queuedValue.x=hoverValue.x;
            queuedValue.y=hoverValue.y;
            }
        }
    }

    @Override
    public void periodic() {
        if (heldObject == HeldObject.CONE) {
            for (int i = 0; i < 2; i++) {
                for (int j = 1; j < 8; j += 3) {
                    nodeSuperStateValues[i][j] = NodeSuperState.INVALID.value;
                }
            }
        } else if (heldObject == HeldObject.CUBE) {
            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 9; j++) {
                    if (j % 3 == 0 || j % 3 == 2) {
                        nodeSuperStateValues[i][j] = NodeSuperState.INVALID.value;
                    }
                }
            }
        } else {
            for (int i = 0; i < 2; i++) {
                for (int j = 1; j < 8; j += 3) {
                    if (nodeSuperStateValues[i][j] == NodeSuperState.INVALID.value)
                        nodeSuperStateValues[i][j] = NodeSuperState.NONE.value;
                }
            }
        }
        for (int i = 0; i < nodes.length; i++) {
            for (int j = 0; j < nodes[i].length; j++) {
                if (nodeSuperStateValues[i][j] == NodeSuperState.NONE.value) {
                    nodes[i][j].setInteger(nodeStateValues[i][j]);
                } else {
                    nodes[i][j].setInteger(nodeSuperStateValues[i][j]);
                }
            }
        }
    }

}
