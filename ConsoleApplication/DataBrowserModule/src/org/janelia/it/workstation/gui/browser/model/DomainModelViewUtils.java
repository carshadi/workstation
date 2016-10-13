package org.janelia.it.workstation.gui.browser.model;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Utilities for extracting information from the domain model for view purposes.
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class DomainModelViewUtils {

    private final static DateFormat dateFormatter = new SimpleDateFormat("yyyy/MM/dd hh:mma");

    public static String getDateString(Date date) {
        return dateFormatter.format(date).toLowerCase();
    }
}
