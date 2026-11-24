package org.opentrafficsim.dcas.object.matrix;

import org.djunits.unit.SpeedUnit;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;
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
    private Historical<MatrixControlState> state;

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
        this.state = new HistoricalValue<>(simulator.getReplication().getHistoryManager(simulator), this,
                new MatrixControlState(MatrixState.BLANK, ControlType.STATIC));
        init();
    }

    /**
     * Sets the matrix state.
     * @param matrixState matrix state
     * @param controlType whether the state is dynamically controlled
     */
    public void setState(final MatrixState matrixState, final ControlType controlType)
    {
        this.state.set(new MatrixControlState(matrixState, controlType));
    }

    /**
     * Returns matrix state.
     * @return matrix state
     */
    public MatrixState getState()
    {
        return this.state.get().matrixState();
    }

    /**
     * Returns matrix state at the given time.
     * @param time time
     * @return matrix state at the given time
     */
    public MatrixState getState(final Duration time)
    {
        return this.state.get(time).matrixState();
    }

    /**
     * Returns matrix state.
     * @return matrix state
     */
    public boolean isDynamicControl()
    {
        return ControlType.DYNAMIC.equals(this.state.get().controlType());
    }

    /**
     * Returns matrix state at the given time.
     * @param time time
     * @return matrix state at the given time
     */
    public boolean isDynamicControl(final Duration time)
    {
        return ControlType.DYNAMIC.equals(this.state.get(time).controlType());
    }

    /**
     * Returns matrix control state.
     * @return matrix control state
     */
    public ControlType getControlType()
    {
        return this.state.get().controlType();
    }

    /**
     * Matrix sign state.
     */
    public enum MatrixState
    {

        /** 50km/h. */
        V50(new Speed(50, SpeedUnit.KM_PER_HOUR)),

        /** 70km/h. */
        V70(new Speed(70, SpeedUnit.KM_PER_HOUR)),

        /** 80km/h. */
        V80(new Speed(80, SpeedUnit.KM_PER_HOUR)),

        /** 90km/h. */
        V90(new Speed(90, SpeedUnit.KM_PER_HOUR)),

        /** 100km/h. */
        V100(new Speed(100, SpeedUnit.KM_PER_HOUR)),

        /** Cross. */
        X(null),

        /** Left arrow. */
        LEFT(null),

        /** Right arrow. */
        RIGHT(null),

        /** Down arrow. */
        DOWN(null),

        /** End of all prohibitions. */
        END(null),

        /** Blank. */
        BLANK(null);

        /** Returns the speed limit. */
        private final Speed speed;

        /**
         * Constructor.
         * @param speed speed limit, or null if not a speed limit
         */
        MatrixState(final Speed speed)
        {
            this.speed = speed;
        }

        /**
         * Returns whether the matrix state is a speed limit.
         * @return whether the matrix state is a speed limit
         */
        public boolean isSpeedLimit()
        {
            return this.speed != null;
        }

        /**
         * Returns the speed limit, or null if not a speed limit.
         * @return the speed limit, or null if not a speed limit
         */
        public Speed getSpeed()
        {
            return this.speed;
        }

    }

    /**
     * Control type.
     */
    public enum ControlType
    {
        /** Dynamic control. */
        DYNAMIC,

        /** Static control, e.g. fixed speed limits during road works or rush hour. */
        STATIC;
    }

    /**
     * Storage of matrix state and control type.
     * @param matrixState matrix state
     * @param controlType control type
     */
    private record MatrixControlState(MatrixState matrixState, ControlType controlType)
    {
    }

}
