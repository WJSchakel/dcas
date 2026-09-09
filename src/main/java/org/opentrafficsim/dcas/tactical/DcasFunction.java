package org.opentrafficsim.dcas.tactical;

import org.opentrafficsim.base.parameters.ParameterException;
import org.opentrafficsim.core.gtu.plan.operational.OperationalPlanException;
import org.opentrafficsim.road.gtu.tactical.TacticalContextEgo;

/**
 * Performs a function of the DCAS system independently from other functions. Interactions with the main system consist of the
 * methods in {@link DcasSystemInterface} and the return value of {@link #apply}. The system will take the most critical result
 * of any DCAS function.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
@FunctionalInterface
public interface DcasFunction
{

    /**
     * Performs a function of the DCAS system independently from other functions. Interactions with the main system consist of
     * the methods in {@link DcasSystemInterface} and the return value.
     * @param context tactical context
     * @param dcas DCAS system
     * @return DCAS function result
     * @throws OperationalPlanException when a perception category is not available
     * @throws ParameterException when a parameter is not available
     */
    DcasFunctionResult apply(TacticalContextEgo context, DcasSystemInterface dcas)
            throws OperationalPlanException, ParameterException;

}
