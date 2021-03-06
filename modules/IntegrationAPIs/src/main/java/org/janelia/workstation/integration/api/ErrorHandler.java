package org.janelia.workstation.integration.api;

/**
 * Implement this to create something capable of handling exceptions at the
 * whole-application level. Call the impl when the code has given up on solving
 * the immediate problem.  This is for "system errors".
 *
 * @author fosterl
 */
public interface ErrorHandler {
    
    String LOOKUP_PATH = "ErrorHandler/Location/Nodes";
    
    void handleException(Throwable ex);

    void handleException(String message, Throwable t);
    
    void handleExceptionQuietly(Throwable t);
    
    void handleExceptionQuietly(String message, Throwable t);
}
