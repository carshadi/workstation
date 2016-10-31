package org.janelia.it.jacs.integration.framework.domain;

import org.janelia.it.jacs.model.domain.DomainObject;


/**
 * Implement this to accept a Domain Object for processing.
 * 
 * @author fosterl
 */
public interface DomainObjectAcceptor extends Compatible<DomainObject> {
    
    public static final String DOMAIN_OBJECT_LOOKUP_PATH = "DomainObject/DomainObjectAcceptor/Nodes";
    
    /**
     * The label for the menu item.
     */
    String getActionLabel();

    /**
     * Should the menu item be shown for the specified domain object?
     */
    @Override
    boolean isCompatible(DomainObject e);
    
    /**
     * Should the menu item be enabled for the specified domain object?
     */
    boolean isEnabled(DomainObject e);
    
    void acceptDomainObject(DomainObject e);
    
    /**
     * Space these apart by at least 100, to leave room for injected separators
     * and for later-stage additions of menu items after the fact.
     * 
     * @return expected ascending order key for this menu item.
     */
    Integer getOrder();
    
    boolean isPrecededBySeparator();
    
    boolean isSucceededBySeparator();
    
}