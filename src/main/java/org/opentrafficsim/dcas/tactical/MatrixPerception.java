package org.opentrafficsim.dcas.tactical;

import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.core.gtu.RelativePosition;
import org.opentrafficsim.core.gtu.perception.AbstractPerceptionCategory;
import org.opentrafficsim.dcas.object.matrix.MatrixSign;
import org.opentrafficsim.road.gtu.LaneBasedGtu;
import org.opentrafficsim.road.gtu.perception.LanePerception;
import org.opentrafficsim.road.gtu.perception.PerceptionCollectable;
import org.opentrafficsim.road.gtu.perception.PerceptionReiterable;
import org.opentrafficsim.road.gtu.perception.RelativeLane;
import org.opentrafficsim.road.gtu.perception.object.PerceivedObject.Kinematics;
import org.opentrafficsim.road.gtu.perception.structure.NavigatingIterable.Entry;

/**
 * MatrixPerception.java.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class MatrixPerception extends AbstractPerceptionCategory<LaneBasedGtu, LanePerception>
        implements MatrixPerceptionCategory
{

    /**
     * Constructor.
     * @param perception perception
     */
    public MatrixPerception(final LanePerception perception)
    {
        super(perception);
    }

    @Override
    public PerceptionCollectable<PerceivedMatrixSign, MatrixSign> getMatrixSigns(final RelativeLane lane)
    {
        return super.computeIfAbsent("matrix", () -> computeMatrixSigns(lane), lane);
    }

    /**
     * Computes the matrix signs as perceived.
     * @param lane lane
     * @return matrix signs
     */
    private PerceptionCollectable<PerceivedMatrixSign, MatrixSign> computeMatrixSigns(final RelativeLane lane)
    {
        try
        {
            Iterable<Entry<MatrixSign>> iterable = getPerception().getLaneStructure().getDownstreamObjects(lane,
                    MatrixSign.class, RelativePosition.FRONT, false);
            return new PerceptionReiterable<>(getGtu(), iterable, (object, distance) ->
            {
                return new PerceivedMatrixSign(object.getId(), Kinematics.staticAhead(distance), object.getState(),
                        object.getControlType());
            });
        }
        catch (ParameterException exception)
        {
            throw new OtsRuntimeException("Unexpected exception while computing first leaders.", exception);
        }
    }

}
