package org.opentrafficsim.dcas.scenario;

import org.djunits.value.vdouble.scalar.Speed;
import org.opentrafficsim.dcas.Serialization;

/**
 * Data container for all assumptions. Any changes to this class need to be reflected in the resource file
 * {@code assumptions.json}.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 * @param penetrationLow low DCAS penetration rate (vehicle has DCAS)
 * @param penetrationHigh high DCAS penetration rate (vehicle has DCAS)
 * @param activationRate rate of activation on vehicles with DCAS
 * @param vGainDcas vGain parameter value for drivers with DCAS activated
 * @param socioDcas socio parameter value for drivers with DCAS activated
 * @param tocNonResponseRate DCAS driver transition-of-control non-response rate
 */
public record Assumptions(double penetrationLow, double penetrationHigh, double activationRate, Speed vGainDcas,
        double socioDcas, double tocNonResponseRate)
{

    /** Singleton instance returned by {@code get()}. */
    private static final Assumptions ASSUMPTIONS = Serialization.fromJsonResource("/assumptions.json", Assumptions.class);

    /**
     * Returns instance from JSON file. This method may be called often as it returns a cached version.
     * @return instance from JSON file
     */
    public static Assumptions get()
    {
        return ASSUMPTIONS;
    }

}
