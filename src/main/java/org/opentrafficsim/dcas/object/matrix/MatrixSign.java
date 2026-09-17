package org.opentrafficsim.dcas.object.matrix;

import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.network.NetworkException;
import org.opentrafficsim.core.perception.Historical;
import org.opentrafficsim.core.perception.HistoricalValue;
import org.opentrafficsim.road.network.Lane;
import org.opentrafficsim.road.network.object.AbstractLaneBasedObject;
import org.opentrafficsim.road.network.object.LaneBasedObject;

/**
 * Matrix sign.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class MatrixSign extends AbstractLaneBasedObject
{

    /** Matrix state. */
    private Historical<MatrixState> state;

    /**
     * Constructor.
     * @param id id
     * @param lane lane
     * @param longitudinalPosition longitudinal position
     * @throws NetworkException on network exception
     */
    public MatrixSign(final String id, final Lane lane, final Length longitudinalPosition) throws NetworkException
    {
        super(id, lane, longitudinalPosition, LaneBasedObject.makeLine(lane, longitudinalPosition), Length.ZERO);
        OtsSimulatorInterface simulator = lane.getLink().getSimulator();
        this.state = new HistoricalValue<>(simulator.getReplication().getHistoryManager(simulator), this, MatrixState.BLANK);
        init();
    }

    /**
     * Sets the matrix state.
     * @param state matrix state
     */
    public void setState(final MatrixState state)
    {
        this.state.set(state);
    }

    /**
     * Returns matrix state.
     * @return matrix state
     */
    public MatrixState getState()
    {
        return this.state.get();
    }

    /**
     * Returns matrix state at the given time.
     * @param time time
     * @return matrix state at the given time
     */
    public MatrixState getState(final Duration time)
    {
        return this.state.get(time);
    }

    /**
     * Matrix sign state.
     */
    public enum MatrixState
    {
        /** 50km/h. */
        V50,

        /** 70km/h. */
        V70,

        /** 80km/h. */
        V80,

        /** 90km/h. */
        V90,

        /** 100km/h. */
        V100,

        /** Cross. */
        X,

        /** Left arrow. */
        LEFT,

        /** Right arrow. */
        RIGHT,

        /** Down arrow. */
        DOWN,

        /** End of all prohibitions. */
        END,

        /** Blank. */
        BLANK;
    }

}
