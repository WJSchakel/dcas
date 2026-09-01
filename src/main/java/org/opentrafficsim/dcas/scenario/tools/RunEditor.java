package org.opentrafficsim.dcas.scenario.tools;

import java.io.IOException;
import java.io.InputStream;
import java.net.URISyntaxException;

import javax.naming.NamingException;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.djutils.io.ResourceResolver;
import org.opentrafficsim.base.logger.Logger;
import org.opentrafficsim.editor.OtsEditor;
import org.opentrafficsim.editor.decoration.DefaultDecorator;
import org.w3c.dom.Document;
import org.xml.sax.SAXException;

import ch.qos.logback.classic.Level;

/**
 * Runs the editor with default decoration and built-in XML schema. This class differs from the OTS version because reading the
 * schema from a resource in a jar does not work as coded there (in 1.8.1). We load local XSD files, and specify a systemId
 * towards the XSD folder in resources.
 * <p>
 * Copyright (c) 2023-2026 Delft University of Technology, PO Box 5, 2600 AA, Delft, the Netherlands. All rights reserved. <br>
 * BSD-style license. See <a href="https://opentrafficsim.org/docs/license.html">OpenTrafficSim License</a>.
 * </p>
 * @author Wouter Schakel
 */
public final class RunEditor
{

    /**
     * Private constructor.
     */
    private RunEditor()
    {

    }

    /**
     * Runs the editor.
     * @param args arguments.
     * @throws IOException exception
     * @throws SAXException exception
     * @throws ParserConfigurationException exception
     * @throws InterruptedException exception
     * @throws URISyntaxException exception
     * @throws NamingException exception
     */
    public static void main(final String[] args) throws IOException, SAXException, ParserConfigurationException,
            InterruptedException, URISyntaxException, NamingException
    {
        Logger.setLogLevel(Level.INFO);
        OtsEditor editor = new OtsEditor();
        DefaultDecorator.decorate(editor);
        editor.setSchema(open(ResourceResolver.resolveAsResource("/xsd/ots.xsd").openStream()));
    }

    /**
     * Opens an XSD or XML file.
     * @param stream stream.
     * @return document, i.e. the root of the XSD file.
     * @throws SAXException exception
     * @throws IOException exception
     * @throws ParserConfigurationException exception
     */
    public static Document open(final InputStream stream) throws SAXException, IOException, ParserConfigurationException
    {
        DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
        dbf.setIgnoringComments(true);
        dbf.setIgnoringElementContentWhitespace(true);
        DocumentBuilder db = dbf.newDocumentBuilder();
        String systemId = ResourceResolver.resolveAsResource("/xsd/").asUri().toString();
        Document doc = db.parse(stream, systemId);
        return doc;
    }

}
