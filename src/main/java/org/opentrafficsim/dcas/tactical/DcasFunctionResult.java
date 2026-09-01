package org.opentrafficsim.dcas.tactical;

/**
 * Result from a DCAS function. Some functions may always return {@code NONE} but internally change something about the
 * behavior of DCAS, for example by setting a lowered speed.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public enum DcasFunctionResult
{

    // Order of constants determines priority for DCAS (last has highest priority).

    /** No specific result for function. */
    NONE,

    /** Change to left lane. */
    CHANGE_LEFT,

    /** Change to right lane. */
    CHANGE_RIGHT,

    /**
     * Request transition of control. The request remains valid until the specific function returns a lower priority result.
     */
    TRANSITION_OF_CONTROL,

    /** Stop in lane. */
    STOP,

    /** Stop on shoulder (system chooses which side). */
    SHOULDER;

}
