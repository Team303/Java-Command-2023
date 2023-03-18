package com.team303.robot.modules;

import static com.team303.robot.Robot.heldObject;

import java.awt.Point;
import java.util.Arrays;

import com.team303.robot.Robot.HeldObject;
import com.team303.robot.util.Alert;
import com.team303.robot.util.Alert.AlertType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Operator extends SubsystemBase {
    public static final ShuffleboardTab OPERATOR_TAB = Shuffleboard.getTab("Operator");
    public static final NetworkTable operator = NetworkTableInstance.getDefault().getTable("Operator");
    public static final SendableChooser<HeldObject> heldObjectChooser = new SendableChooser<HeldObject>();
    public static final GenericEntry[][] nodes = new GenericEntry[3][9];
    public static GenericEntry hpSuggestion;
    public static final int[][] nodeStateValues = new int[3][9];
    public static final int[][] nodeSuperStateValues = new int[3][9];
    public static boolean[] linkComplete = new boolean[21];
    public static boolean coopertitionBonusAchieved;
    public boolean queueManualOverride = false;
    public boolean suggestManualOverride = false;
    public Point hoverValue = new Point(0, 0);
    public Point queuedValue;
    private final Alert logQueueOnFilledNode = new Alert("Operator Terminal",
            "Attempted to queue on already-filled node, queue not performed", AlertType.WARNING);
    private final Alert logNoMorePieceSpaceCones = new Alert("Operator Terminal",
            "No more space to place cones, queue canceled", AlertType.WARNING);
    private final Alert logNoMorePieceSpaceCubes = new Alert("Operator Terminal",
            "No more space to place cubes, queue canceled", AlertType.WARNING);

    public static enum NodeState {
        NONE(0),
        CONE(1),
        CUBE(2);

        public final int value;

        private NodeState(int value) {
            this.value = value;
        }
    }

    public static enum NodeSuperState {
        NONE(0),
        HOVER(3),
        QUEUED(4),
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
        heldObjectChooser.addOption("None", HeldObject.NONE);
        heldObjectChooser.addOption("Cube", HeldObject.CUBE);
        heldObjectChooser.addOption("Cone", HeldObject.CONE);
        OPERATOR_TAB.add("Held Object Chooser", heldObjectChooser).withPosition(2, 0);
        hpSuggestion = OPERATOR_TAB.add("HP Suggestion", 0).withPosition(8, 0).withWidget("State of Node").getEntry();
        nodeSuperStateValues[0][0] = NodeSuperState.HOVER.value;
    }

    public void moveDown() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        for (int k = 1; k < nodeSuperStateValues.length + 1; k++) {
            int newRow = hoverValue.x + k;
            if (newRow > 2) {
                newRow -= 3;
            }
            if (nodeSuperStateValues[newRow][hoverValue.y] == NodeSuperState.NONE.value
                    || nodeSuperStateValues[newRow][hoverValue.y] == NodeSuperState.QUEUED.value) {
                hoverValue.x = newRow;
                nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
                return;
            }
        }
    }

    public void moveUp() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        for (int k = 1; k < nodeSuperStateValues.length + 1; k++) {
            int newRow = hoverValue.x - k;
            if (newRow < 0) {
                newRow += 3;
            }
            if (nodeSuperStateValues[newRow][hoverValue.y] == NodeSuperState.NONE.value
                    || nodeSuperStateValues[newRow][hoverValue.y] == NodeSuperState.QUEUED.value) {
                nodeSuperStateValues[newRow][hoverValue.y] = NodeSuperState.HOVER.value;
                hoverValue.x = newRow;
                return;
            }
        }
    }

    public void moveLeft() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        for (int k = 1; k < nodeSuperStateValues[hoverValue.x].length + 1; k++) {
            int newCol = hoverValue.y - k;
            if (newCol < 0) {
                newCol += 9;
            }
            if (nodeSuperStateValues[hoverValue.x][newCol] == NodeSuperState.NONE.value
                    || nodeSuperStateValues[hoverValue.x][newCol] == NodeSuperState.QUEUED.value) {
                nodeSuperStateValues[hoverValue.x][newCol] = NodeSuperState.HOVER.value;
                hoverValue.y = newCol;
                return;
            }
        }
    }

    public void moveRight() {
        if (!hoverValue.equals(queuedValue)) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.NONE.value;
        } else {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
        }
        for (int k = 1; k < nodeSuperStateValues[hoverValue.x].length + 1; k++) {
            int newCol = hoverValue.y + k;
            if (newCol > 8) {
                newCol -= 9;
            }
            if (nodeSuperStateValues[hoverValue.x][newCol] == NodeSuperState.NONE.value
                    || nodeSuperStateValues[hoverValue.x][newCol] == NodeSuperState.QUEUED.value) {
                nodeSuperStateValues[hoverValue.x][newCol] = NodeSuperState.HOVER.value;
                hoverValue.y = newCol;
                return;
            }
        }
    }

    public void manualSuggest() {
        int state = (int) hpSuggestion.getInteger(0);
        if (state == 2) {
            hpSuggestion.setInteger(--state);
        } else {
            hpSuggestion.setInteger(++state);
        }
        suggestManualOverride = true;
    }

    public void setPiece() {
        if (nodeStateValues[hoverValue.x][hoverValue.y] == NodeState.CUBE.value
                || nodeStateValues[hoverValue.x][hoverValue.y] == NodeState.CONE.value) {
            nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.NONE.value;
            autoQueuePlacement();
        } else if (hoverValue.x > 1) {
            nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.CUBE.value;
            autoQueuePlacement();
        } else if (hoverValue.y % 3 == 0 || hoverValue.y % 3 == 2) {
            nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.CONE.value;
            autoQueuePlacement();
        } else {
            nodeStateValues[hoverValue.x][hoverValue.y] = NodeState.CUBE.value;
            autoQueuePlacement();
        }
        if (heldObject == HeldObject.NONE) {
            autoSuggestPiece();
        }
    }

    public void queuePlacement() {
        if (nodeSuperStateValues[hoverValue.x][hoverValue.y] == NodeSuperState.QUEUED.value) {
            nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.HOVER.value;
            queueManualOverride = false;
            queuedValue = null;
        } else {
            if (nodeStateValues[hoverValue.x][hoverValue.y] != NodeState.NONE.value) {
                logQueueOnFilledNode.set(true);
                return;
            }
            logQueueOnFilledNode.set(false);
            if (queuedValue == null) {
                nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
                queueManualOverride = true;
                queuedValue = new Point(hoverValue);
            } else {
                nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
                queueManualOverride = true;
                nodeSuperStateValues[hoverValue.x][hoverValue.y] = NodeSuperState.QUEUED.value;
                queuedValue.x = hoverValue.x;
                queuedValue.y = hoverValue.y;
            }
        }
    }
    private boolean partOfCompleteLink(int i, int j) {
        int baseNineIndex = 9*i+j;
        if (baseNineIndex%9==0) {
            return linkComplete[i*7];
        } else if (baseNineIndex%9==1) {
            // System.out.println(linkComplete[i*7+1]);
            // System.out.println(linkComplete[i*7]);
            return (linkComplete[i*7+1] || linkComplete[i*7]);
        } else if (baseNineIndex%9>=2 && baseNineIndex%9<=6) {
            //System.out.println(baseNineIndex);
            return linkComplete[i*7+(j-2)] || linkComplete[i*7+j-1] || linkComplete[i*7+j];
        } else if (baseNineIndex%9==7) {
            return (linkComplete[i*7+j-2] || linkComplete[i*7+j-1]);
        } else {
            return linkComplete[i*7+6];
        }
    }

    public void autoSuggestPiece() {
        if (suggestManualOverride || heldObject != HeldObject.NONE) {
            return;
        }
        // First priority is to complete coopertition bonus
        if (!coopertitionBonusAchieved) {
            for (int i = 0; i < 3; i++) {
                if ((nodeStateValues[i][3] != NodeState.NONE.value
                        && nodeStateValues[i][4] != NodeState.NONE.value)
                        || (nodeStateValues[i][4] != NodeState.NONE.value
                                && nodeStateValues[i][5] != NodeState.NONE.value)) {
                    hpSuggestion.setInteger(NodeState.CONE.value);
                    System.out.println(0);
                    return;
                } else if ((nodeStateValues[i][3] != NodeState.NONE.value
                        && nodeStateValues[i][5] != NodeState.NONE.value)) {
                    hpSuggestion.setInteger(NodeState.CUBE.value);
                    System.out.println(1);
                    return;

                }
            }
        }
        // Second priority is to complete a link
        for (int i = 0; i < 3; i++) {
            if ((nodeStateValues[i][0] != NodeState.NONE.value
                    && nodeStateValues[i][1] != NodeState.NONE.value)
                    && nodeStateValues[i][2] == NodeState.NONE.value) {
                if (i == 2) {
                    hpSuggestion.setInteger(NodeState.CUBE.value);
                    System.out.println(2);
                    return;
                }
                hpSuggestion.setInteger(NodeState.CONE.value);
                return;
            } else if ((nodeStateValues[i][1] != NodeState.NONE.value
                    && nodeStateValues[i][2] != NodeState.NONE.value)
                    && nodeStateValues[i][0] == NodeState.NONE.value) {
                if (i == 2) {
                    hpSuggestion.setInteger(NodeState.CUBE.value);
                    System.out.println(3);
                    return;
                }
                hpSuggestion.setInteger(NodeState.CONE.value);
                System.out.println(4);
                return;
            } else if ((nodeStateValues[i][0] != NodeState.NONE.value
                    && nodeStateValues[i][2] != NodeState.NONE.value)
                    && nodeStateValues[i][1] == NodeState.NONE.value) {
                if (i == 2) {
                    hpSuggestion.setInteger(NodeState.CUBE.value);
                    System.out.println(5);
                    return;
                }
                hpSuggestion.setInteger(NodeState.CUBE.value);
                System.out.println(6);
                return;
            }
            for (int j = 1; j < 5; j++) {
                if ((nodeStateValues[i][j] != NodeState.NONE.value
                        && nodeStateValues[i][j + 1] != NodeState.NONE.value)
                        && nodeStateValues[i][j + 2] == NodeState.NONE.value && nodeStateValues[i][j -1] == NodeState.NONE.value) {
                    if (i == 2) {
                        hpSuggestion.setInteger(NodeState.CUBE.value);
                        System.out.println(7);
                        return;
                    }
                    if ((j + 2) % 3 == 1) {
                        hpSuggestion.setInteger(NodeState.CUBE.value);
                        System.out.println(8);
                        return;
                    }
                    hpSuggestion.setInteger(NodeState.CONE.value);
                    System.out.println(9);
                    return;
                } else if ((nodeStateValues[i][j + 1] != NodeState.NONE.value
                        && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                        && nodeStateValues[i][j] == NodeState.NONE.value && nodeStateValues[i][j + 3] == NodeState.NONE.value) {
                    if (i == 2) {
                        hpSuggestion.setInteger(NodeState.CUBE.value);
                        System.out.println(10);
                        return;
                    }
                    if (j % 3 == 1) {
                        hpSuggestion.setInteger(NodeState.CUBE.value);
                        System.out.println(11);
                        return;
                    }
                    hpSuggestion.setInteger(NodeState.CONE.value);
                    System.out.println(12);
                    return;
                } else if ((nodeStateValues[i][j] != NodeState.NONE.value
                        && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                        && nodeStateValues[i][j + 1] == NodeState.NONE.value) {
                    if (i == 2) {
                        hpSuggestion.setInteger(NodeState.CUBE.value);
                        System.out.println(13);
                        return;
                    }
                    if ((j + 1) % 3 == 1) {
                        hpSuggestion.setInteger(NodeState.CUBE.value);
                        System.out.println(14);
                        return;
                    }
                    hpSuggestion.setInteger(NodeState.CONE.value);
                    System.out.println(15);
                    return;
                }
            }
            
        if ((nodeStateValues[i][6] != NodeState.NONE.value
        && nodeStateValues[i][7] != NodeState.NONE.value)
        && nodeStateValues[i][8] == NodeState.NONE.value) {
    if (i == 2) {
        hpSuggestion.setInteger(NodeState.CUBE.value);
        System.out.println(16);
        return;
    }
    hpSuggestion.setInteger(NodeState.CONE.value);
    return;
} else if ((nodeStateValues[i][7] != NodeState.NONE.value
        && nodeStateValues[i][8] != NodeState.NONE.value)
        && nodeStateValues[i][6] == NodeState.NONE.value) {
    if (i == 2) {
        hpSuggestion.setInteger(NodeState.CUBE.value);
        System.out.println(17);
        return;
    }
    hpSuggestion.setInteger(NodeState.CONE.value);
    return;
} else if ((nodeStateValues[i][6] != NodeState.NONE.value
        && nodeStateValues[i][8] != NodeState.NONE.value)
        && nodeStateValues[i][7] == NodeState.NONE.value) {
    if (i == 2) {
        hpSuggestion.setInteger(NodeState.CUBE.value);
        System.out.println(18);
        return;
    }
    hpSuggestion.setInteger(NodeState.CUBE.value);
    System.out.println(19);
    return;
}
            
        }
        // Otherwise, there is no valid reason to choose between cones and cubes. Random
        // choice
        hpSuggestion.setInteger((int) (Math.random() * 2) + 1);
        System.out.println(20);
    }

    public void autoQueuePlacement() {
        if (queueManualOverride) {
            return;
        }
        if (queuedValue != null) {
            nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
        }
        if (heldObject == HeldObject.CONE) {
            // First priority is to achieve coopertition bonus link (all priorities
            // automatically go for highest possible)
            if (!coopertitionBonusAchieved) {
                for (int i = 0; i < 3; i++) {
                    int j = 3;
                    if ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 1] != NodeState.NONE.value)) {
                        queuedValue = new Point(i, j + 2);
                        nodeSuperStateValues[i][j + 2] = NodeSuperState.QUEUED.value;
                        return;
                    } else if ((nodeStateValues[i][j + 1] != NodeState.NONE.value
                            && nodeStateValues[i][j + 2] != NodeState.NONE.value)) {
                        queuedValue = new Point(i, j);
                        nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
                        return;
                    } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 1] != NodeState.NONE.value)
                            || (nodeStateValues[i][j + 1] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                            || (nodeStateValues[i][j] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value))) {
                        for (int k = 0; k < 3; k++) {
                            if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
                                queuedValue = new Point(i, j + k);
                                nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
                                return;
                            }
                        }
                    }
                }
            }
            // Next priority is to complete a link
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 8; j += 3) {
                    if ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 1] != NodeState.NONE.value)
                            && nodeStateValues[i][j + 2] == NodeState.NONE.value) {
                        queuedValue = new Point(i, j + 2);
                        nodeSuperStateValues[i][j + 2] = NodeSuperState.QUEUED.value;
                        return;
                    } else if ((nodeStateValues[i][j + 1] != NodeState.NONE.value
                            && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                            && nodeStateValues[i][j] == NodeState.NONE.value) {
                        queuedValue = new Point(i, j);
                        nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
                        return;
                    } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 1] != NodeState.NONE.value)
                            || (nodeStateValues[i][j + 1] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                            || (nodeStateValues[i][j] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value))) {
                        for (int k = 0; k < 3; k++) {
                            if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
                                queuedValue = new Point(i, j + k);
                                nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
                                return;
                            }
                        }
                    }
                }
            }
            // Next priority is to make 2/3 link
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 8; j += 3) {
                    // Case 1: Cube node in link is filled
                    if (nodeStateValues[i][(j / 3) * 3 + 1] != NodeState.NONE.value
                            && nodeStateValues[i][j] == NodeState.NONE.value) {
                        queuedValue = new Point(i, j);
                        nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
                        return;
                        // Case 2: First cone node in link is filled
                    } else if (nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 2] == NodeState.NONE.value) {
                        queuedValue = new Point(i, j + 2);
                        nodeSuperStateValues[i][j + 2] = NodeSuperState.QUEUED.value;
                        return;
                        // Case 3: Second cone node in link is filled
                    } else if (nodeStateValues[i][j] == NodeState.NONE.value
                            && nodeStateValues[i][j + 2] != NodeState.NONE.value) {
                        queuedValue = new Point(i, j);
                        nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
                        return;
                    } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 1] != NodeState.NONE.value)
                            || (nodeStateValues[i][j + 1] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                            || (nodeStateValues[i][j] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value))) {
                        for (int k = 0; k < 3; k++) {
                            if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
                                queuedValue = new Point(i, j + k);
                                nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
                                return;
                            }
                        }
                    }
                }
            }
            // Last priority is to just place wherever empty
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 8; j++) {
                    if (nodeStateValues[i][j] == NodeState.NONE.value && (j % 3 == 0 || j % 3 == 2)) {
                        queuedValue = new Point(i, j);
                        nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
                        return;
                    }
                }
            }
            logNoMorePieceSpaceCones.set(true);
            if (queuedValue != null) {
                nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
            }
            queuedValue = null;
        } else if (heldObject == HeldObject.CUBE) {
            // First priority is to achieve coopertition bonus link (all priorities
            // automatically go for highest possible)
            if (!coopertitionBonusAchieved) {
                for (int i = 0; i < 3; i++) {
                    int j = 3;
                    if ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 2] != NodeState.NONE.value)) {
                        queuedValue = new Point(i, j + 1);
                        nodeSuperStateValues[i][j + 1] = NodeSuperState.QUEUED.value;
                        return;
                    } else if (i == 2 && ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 1] != NodeState.NONE.value)
                            || (nodeStateValues[i][j + 1] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                            || (nodeStateValues[i][j] != NodeState.NONE.value
                                    && nodeStateValues[i][j + 2] != NodeState.NONE.value))) {
                        for (int k = 0; k < 3; k++) {
                            if (nodeStateValues[i][j + k] == NodeState.NONE.value) {
                                queuedValue = new Point(i, j + k);
                                nodeSuperStateValues[i][j + k] = NodeSuperState.QUEUED.value;
                                return;
                            }
                        }
                    }
                }
            }
            // Next priority is to complete a link
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 8; j += 3) {
                    if ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][j + 2] != NodeState.NONE.value)
                            && nodeStateValues[i][j + 1] == NodeState.NONE.value) {
                        queuedValue = new Point(i, j + 1);
                        nodeSuperStateValues[i][j + 1] = NodeSuperState.QUEUED.value;
                        return;
                    }
                }
            }
            // Next priority is to make 2/3 link
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 8; j++) {
                    if ((nodeStateValues[i][j] != NodeState.NONE.value
                            && nodeStateValues[i][(j / 3) * 3 + 1] == NodeState.NONE.value)) {
                        queuedValue = new Point(i, (j / 3) * 3 + 1);
                        nodeSuperStateValues[i][(j / 3) * 3 + 1] = NodeSuperState.QUEUED.value;
                        return;
                    }
                }
            }
            // Last priority is to just place wherever empty
            for (int i = 0; i < 3; i++) {
                for (int j = 1; j < 8; j += 3) {
                    if (nodeStateValues[i][j] == NodeState.NONE.value && j % 3 == 1) {
                        queuedValue = new Point(i, j);
                        nodeSuperStateValues[i][j] = NodeSuperState.QUEUED.value;
                        return;
                    }
                }
            }
            logNoMorePieceSpaceCubes.set(true);
            if (queuedValue != null) {
                nodeSuperStateValues[queuedValue.x][queuedValue.y] = NodeSuperState.NONE.value;
            }
            queuedValue = null;
        } else {
            return;
        }
    }

    @Override
    public void periodic() {
        //Check which links are complete
        for (int i=0;i<3;i++) {
            for (int j=1;j<8;) {
                if (nodeStateValues[i][j]!=NodeState.NONE.value && nodeStateValues[i][j+1]!=NodeState.NONE.value && nodeStateValues[i][j-1]!=NodeState.NONE.value) {                 
                    linkComplete[7*i+(j-1)]=true;
                    if (j>1 && linkComplete[7*i+(j-2)]) {
                        linkComplete[7*i+(j-2)]=false;
                    }
                    if (j>2 && linkComplete[7*i+(j-3)]) {
                        linkComplete[7*i+(j-3)]=false;
                    }
                    if (j<4) {
                    j=((j-1)/3+1)*3+1;
                    }else {
                        j=8;
                    }
                } else {
                    linkComplete[7*i+(j-1)]=false;
                    j++;
                }
            }
        }   
        if (heldObject != heldObjectChooser.getSelected()) {
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 9; j++) {
                    if (nodeSuperStateValues[i][j] == NodeSuperState.INVALID.value
                            || nodeSuperStateValues[i][j] == NodeSuperState.QUEUED.value) {
                        nodeSuperStateValues[i][j] = NodeSuperState.NONE.value;
                    }
                }
            }
            heldObject = heldObjectChooser.getSelected();
            autoQueuePlacement();
        }
        if (suggestManualOverride && heldObject != HeldObject.NONE) {
            suggestManualOverride = false;
        }
        for (int i = 0; i < 3; i++) {
            if ((nodeStateValues[i][3] != NodeState.NONE.value && nodeStateValues[i][4] != NodeState.NONE.value
                    && nodeStateValues[i][5] != NodeState.NONE.value)) {
                coopertitionBonusAchieved = true;
                break;
            }
        }
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
                for (int j = 0; j < 9; j++) {
                    if (nodeSuperStateValues[i][j] == NodeSuperState.INVALID.value) {
                        nodeSuperStateValues[i][j] = NodeSuperState.NONE.value;
                    }
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
