package org.opentrafficsim.dcas;

import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Locale;

import org.djunits.value.vdouble.scalar.Speed;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;

import nl.tudelft.simulation.dsol.swing.gui.util.Resource;

/**
 * Standard GSON serialization.
 * <p>
 * Copyright (c) 2026-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved.<br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * @author Wouter Schakel
 * @author Saeed Rahmani
 */
public final class Serialization
{

    /** Speed adapter. */
    private static final TypeAdapter<Speed> SPEED_ADAPTER = new TypeAdapter<Speed>()
    {
        @Override
        public void write(final JsonWriter out, final Speed value) throws IOException
        {
            // DJUNITS 6.0.1 has much better support for formatting
            Locale locale = Locale.getDefault();
            Locale.setDefault(Locale.US); // use . decimal point
            out.value(value.toString());
            Locale.setDefault(locale);
        }

        @Override
        public Speed read(final JsonReader in) throws IOException
        {
            return Speed.valueOf(in.nextString()); // uses . decimal point
        }
    };

    /** Instance of {@Gson} with all adapters loaded. */
    public static final Gson GSON = new GsonBuilder().registerTypeAdapter(Speed.class, SPEED_ADAPTER).create();

    /** Constructor. */
    private Serialization()
    {
        // utility class
    }

    /**
     * Returns deserialized instance of class from reader.
     * @param <T> type class
     * @param reader reader
     * @param clazz type class
     * @return deserialized instance of class from reader
     */
    public static <T> T fromJsonReader(final JsonReader reader, final Class<T> clazz)
    {
        return GSON.fromJson(reader, clazz);
    }

    /**
     * Returns deserialized instance of class from resource file (in src/main/resources).
     * @param <T> type class
     * @param resource resource file compliant to {@link Resource#getResourceAsStream(String)}. For plain files this should
     *            start with {@code "/"}.
     * @param clazz type class
     * @return deserialized instance of class from resource file
     */
    public static <T> T fromJsonResource(final String resource, final Class<T> clazz)
    {
        return fromJsonReader(new JsonReader(new InputStreamReader(Resource.getResourceAsStream(resource))), clazz);
    }

}
