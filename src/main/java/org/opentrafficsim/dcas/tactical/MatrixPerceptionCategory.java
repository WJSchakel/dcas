package org.opentrafficsim.dcas.tactical;

import org.djunits.value.vdouble.scalar.Length;
import org.opentrafficsim.dcas.object.matrix.MatrixSign;
import org.opentrafficsim.dcas.object.matrix.MatrixSign.ControlType;
import org.opentrafficsim.dcas.object.matrix.MatrixSign.MatrixState;
import org.opentrafficsim.road.gtu.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
import org.opentrafficsim.road.gtu.perception.categories.LaneBasedPerceptionCategory;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObject;

/**
 * Perception of matrix signs.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public interface MatrixPerceptionCategory extends LaneBasedPerceptionCategory
{

    /**
     * Returns the matrix signs as perceived.
     * @param lane lane
     * @return matrix signs
     */
    PerceptionCollectable<PerceivedMatrixSign, MatrixSign> getMatrixSigns(RelativeLane lane);

    /**
     * Matrix sign as perceived.
     * @param getId id of the matrix sign
     * @param getKinematics kinematics of the matrix sign
     * @param matrixState state displayed on the matrix sign
     * @param controlType whether the state is dynamically controlled
     */
    record PerceivedMatrixSign(String getId, Kinematics getKinematics, MatrixState matrixState, ControlType controlType)
            implements PerceivedObject
    {

        @Override
        public ObjectType getObjectType()
        {
            return ObjectType.OBJECT;
        }

        @Override
        public Length getLength()
        {
            return Length.ZERO;
        }

    }

}
