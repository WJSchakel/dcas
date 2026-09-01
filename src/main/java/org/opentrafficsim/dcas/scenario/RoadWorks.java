package org.opentrafficsim.dcas.scenario;

import java.util.Collections;
import java.util.List;

import org.djunits.unit.DurationUnit;
import org.djunits.unit.FrequencyUnit;
import org.djunits.value.vdouble.scalar.Frequency;
import org.djunits.value.vdouble.vector.DurationVector;
import org.djunits.value.vdouble.vector.FrequencyVector;
import org.djutils.cli.CliUtil;
import org.djutils.immutablecollections.ImmutableMap;
import org.opentrafficsim.base.OtsRuntimeException;
import org.opentrafficsim.core.definitions.Definitions;
import org.opentrafficsim.core.dsol.OtsSimulatorInterface;
import org.opentrafficsim.core.gtu.GtuType;
import org.opentrafficsim.core.network.Node;
import org.opentrafficsim.dcas.scenario.tools.ModelSetup;
import org.opentrafficsim.dcas.tactical.Dcas;
import org.opentrafficsim.road.network.RoadNetwork;
import org.opentrafficsim.road.network.factory.xml.parser.XmlParser;
import org.opentrafficsim.road.od.Categorization;
import org.opentrafficsim.road.od.Category;
import org.opentrafficsim.road.od.Interpolation;
import org.opentrafficsim.road.od.OdMatrix;
import org.opentrafficsim.swing.gui.OtsSimulationPanelDecorator;
import org.opentrafficsim.swing.script.AbstractSimulationScript;

import nl.tudelft.simulation.dsol.SimRuntimeException;
import picocli.CommandLine.Option;

/**
 * RoadWorks scenario.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public class RoadWorks extends AbstractSimulationScript
{

    /** XML scenario. */
    @Option(names = "--xmlScenario", description = "Scenario in XML, if any")
    private String xmlScenario;

    /** Number of lanes. */
    @Option(names = "--lanes", description = "Number of lanes, 2 or 3", defaultValue = "3")
    private int lanes;

    /** Truck fraction. */
    @Option(names = "--fTruck", description = "Truck fraction: 0 <= fTruck <= 1", defaultValue = "0.1")
    private double fTruck;

    /** DCAS fraction. */
    @Option(names = "--fDcas", description = "DCAS fraction (of cars): 0 <= fDcas <= 1", defaultValue = "0.2")
    private double fDcas;

    /** DCAS can change lane. */
    @Option(names = "--lcDcas", description = "DCAS can change lane", defaultValue = "true")
    private boolean lcDcas;

    /** DCAS can move to shoulder. */
    @Option(names = "--shoulderDcas", description = "DCAS can move to shoulder", defaultValue = "true")
    private boolean shoulderDcas;

    /** Peak demand. */
    @Option(names = "--demand", description = "Peak demand", defaultValue = "4000/h")
    private Frequency demand;

    /** GTU types mapped from ID to object. */
    private ImmutableMap<String, GtuType> gtuTypes;

    /**
     * Constructor.
     */
    protected RoadWorks()
    {
        super("RoadWorks", "Scenario with DCAS transition in to roadworks (i.e. possibly unknown lane drop)");
    }

    /**
     * Main method.
     * @param args String[] command line arguments
     * @throws Exception any exception
     */
    public static void main(final String[] args) throws Exception
    {
        RoadWorks scenario = new RoadWorks();
        CliUtil.execute(scenario, args);
        Dcas.setSetting(Dcas.LC_DCAS, scenario.lcDcas);
        Dcas.setSetting(Dcas.SHOULDER_DCAS, scenario.shoulderDcas);
        scenario.start();
    }

    @Override
    protected OtsSimulationPanelDecorator getDecorator()
    {
        return ModelSetup.getDecorator(this.gtuTypes);
    }

    @Override
    protected RoadNetwork setupSimulation(final OtsSimulatorInterface sim) throws Exception
    {
        String resource = "/resources/xml/lanedrop" + this.lanes + "-" + (this.lanes - 1) + ".xml";
        RoadNetwork network = new RoadNetwork("RoadWorks", getSimulator());
        try
        {
            XmlParser xmlParser = new XmlParser(network).setResource(resource).setScenario(this.xmlScenario);
            Definitions definitions = xmlParser.build().definitions();
            this.gtuTypes = definitions.getAll(GtuType.class);

            GtuType car = this.gtuTypes.get("CAR");
            GtuType dcas = this.gtuTypes.get("DCAS");
            GtuType truck = this.gtuTypes.get("TRUCK");

            Node nodeA = network.getNode("A").get();
            Node nodeD = network.getNode("D").get();

            /*
             * The categorization is how demand is further specified beyond just O-D. For example per GTU type. The
             * categorization can also be extended with Route.class, for example new Categorization("categorization",
             * GtuType.class, Route.class). Each instance of a category then should be specified including a route, e.g. new
             * Category(categorization, car, routeAD).
             */
            /*
             * A demand pattern (DurationVector and FrequencyVector) can be reused for different (O, D, category) combinations
             * by also providing a double as fraction. For example: odMatrix.putDemandVector(..., demand, 0.8). Although a
             * global duration vector is provided, specific (O, D, category) combinations can use their custom demand pattern.
             * The same holds for how the demand pattern is interpolated. For example: odMatrix.putDemandVector(..., demand,
             * durationVector, Interpolation.STEPWISE, 0.7).
             */
            List<Node> origins = Collections.singletonList(nodeA);
            List<Node> destinations = Collections.singletonList(nodeD);
            Categorization categorization = new Categorization("categorization", GtuType.class);
            DurationVector globalDurationVector = new DurationVector(new double[] {0, 20, 60}, DurationUnit.MINUTE);
            Interpolation globalInterPolation = Interpolation.LINEAR;
            OdMatrix odMatrix =
                    new OdMatrix("od", origins, destinations, categorization, globalDurationVector, globalInterPolation);

            FrequencyVector demandVector = new FrequencyVector(
                    new double[] {this.demand.si * .5, this.demand.si, this.demand.si * .5}, FrequencyUnit.SI);

            Category carCategory = new Category(categorization, car); // can add Route.class instance
            odMatrix.putDemandVector(nodeA, nodeD, carCategory, demandVector, (1.0 - this.fTruck) * (1.0 - this.fDcas));
            Category dcasCategory = new Category(categorization, dcas);
            odMatrix.putDemandVector(nodeA, nodeD, dcasCategory, demandVector, (1.0 - this.fTruck) * this.fDcas);
            Category truckCategory = new Category(categorization, truck);
            odMatrix.putDemandVector(nodeA, nodeD, truckCategory, demandVector, this.fTruck);

            ModelSetup.applyOd(network, xmlParser, definitions, odMatrix);

            // TODO Sampler

        }
        catch (SimRuntimeException exception)
        {
            throw new OtsRuntimeException("Unable to load or parse XML file.", exception);
        }
        return network;
    }

}
