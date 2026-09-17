package org.opentrafficsim.dcas.object.matrix;

import org.opentrafficsim.animation.data.AnimationIdentifiableShape;
import org.opentrafficsim.dcas.object.matrix.MatrixSign.MatrixState;
import org.opentrafficsim.dcas.object.matrix.MatrixSignAnimation.MatrixSignData;

/**
 * Wraps a matrix sign and represents it as animation data.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class AnimationMatrixData extends AnimationIdentifiableShape<MatrixSign> implements MatrixSignData
{

    /**
     * Constructor.
     * @param matrix matrix sign
     */
    public AnimationMatrixData(final MatrixSign matrix)
    {
        super(matrix);
    }

    @Override
    public MatrixState getState()
    {
        return getObject().getState();
    }

    @Override
    public String toString()
    {
        return "Matrix " + getObject().getFullId();
    }

}
