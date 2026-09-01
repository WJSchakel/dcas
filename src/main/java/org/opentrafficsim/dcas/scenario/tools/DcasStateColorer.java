package org.opentrafficsim.dcas.scenario.tools;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import org.opentrafficsim.animation.Colors;
import org.opentrafficsim.animation.colorer.LegendColorer;
import org.opentrafficsim.core.gtu.Gtu;
import org.opentrafficsim.dcas.tactical.DcasTacticalPlanner;

/**
 * Colorer for the DCAS state.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class DcasStateColorer implements LegendColorer<Gtu>
{

    /** Off. */
    static final Color OFF = Colors.OTS_BLUE;

    /** On. */
    static final Color ON = Color.GREEN.darker();

    /** Transition Of Control request. */
    static final Color TOC = Color.YELLOW;

    /** Minimum Risk Maneuver. */
    static final Color MRM = Color.RED;

    /** Not available. */
    static final Color NA = Color.WHITE;

    /** Legend. */
    static final List<LegendEntry> LEGEND;

    static
    {
        LEGEND = new ArrayList<>();
        LEGEND.add(new LegendEntry(OFF, "off", "off"));
        LEGEND.add(new LegendEntry(ON, "on", "on, normal operations"));
        LEGEND.add(new LegendEntry(TOC, "transition of control", "transition of control"));
        LEGEND.add(new LegendEntry(MRM, "minimum risk maneuver", "minimum risk maneuver"));
        LEGEND.add(new LegendEntry(NA, "N/A", "N/A"));
    }

    @Override
    public Color getColor(final Gtu object)
    {
        if (object.getTacticalPlanner() instanceof DcasTacticalPlanner planner)
        {
            switch (planner.getState())
            {
                case OFF:
                    return OFF;
                case ON:
                    return ON;
                case TOC:
                    return TOC;
                case MRM:
                    return MRM;
                default:
                    return NA;
            }
        }
        return NA;
    }

    @Override
    public List<LegendEntry> getLegend()
    {
        return LEGEND;
    }

    @Override
    public String getName()
    {
        return "DCAS activation";
    }

}
