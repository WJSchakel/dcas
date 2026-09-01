package org.opentrafficsim.dcas.scenario.tools;

import org.djunits.value.vdouble.scalar.Acceleration;
import org.djunits.value.vdouble.scalar.Duration;
import org.djunits.value.vdouble.scalar.Length;
import org.djunits.value.vdouble.scalar.Speed;

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
 * @param s0Dcas DCAS IDM car-following stopping distance
 * @param TDcas DCAS IDM car-following time headway
 * @param aDcas DCAS IDM car-following acceleration parameter
 * @param bDcas DCAS IDM car-following deceleration parameter
 * @param b0Dcas DCAS IDM car-following adjustment deceleration parameter
 * @param deltaDcas DCAS IDM car-following delta parameter
 * @param maxBDcas DCAS maximum deceleration
 * @param minTtcDcas DCAS LC minimum TTC
 * @param minTDcas DCAS LC minimum time headway
 * @param bStopDcas deceleration for in-lane minimum risk maneuver
 * @param dtDcas DCAS system time step
 * @param vGainDcas vGain parameter value for drivers with DCAS activated
 * @param socioDcas socio parameter value for drivers with DCAS activated
 * @param tocNonResponseRate DCAS driver transition-of-control non-response rate
 */
public record Assumptions(double penetrationLow, double penetrationHigh, double activationRate, Length s0Dcas, Duration TDcas,
        Acceleration aDcas, Acceleration bDcas, Acceleration b0Dcas, double deltaDcas, Acceleration maxBDcas,
        Duration minTtcDcas, Duration minTDcas, Acceleration bStopDcas, Duration dtDcas, Speed vGainDcas, double socioDcas,
        double tocNonResponseRate)
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
