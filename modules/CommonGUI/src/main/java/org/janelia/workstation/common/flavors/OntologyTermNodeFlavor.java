package org.janelia.workstation.common.flavors;

import java.awt.datatransfer.DataFlavor;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Drop flavors for ontology terms.
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class OntologyTermNodeFlavor extends DataFlavor {
    
    private final static Logger log = LoggerFactory.getLogger(OntologyTermNodeFlavor.class);
    
    static private DataFlavor createConstant(String mt, String prn) {
        try {
            return new OntologyTermNodeFlavor(mt, prn);
        } catch (Exception e) {
            log.error("Error creating data flavor "+mt,e);
            return null;
        }
    }
    
    public static final DataFlavor SINGLE_FLAVOR = createConstant("application/x-domain-object;class=org.janelia.workstation.browser.nodes.OntologyTermNode","Ontology Term Node");
    public static final DataFlavor LIST_FLAVOR = createConstant("application/x-domain-object-list;class=java.util.List","Ontology Term Node List");

    private OntologyTermNodeFlavor(String mt, String prn) {
        super(mt, prn);
    }

}