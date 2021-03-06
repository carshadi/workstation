package org.janelia.workstation.integration.api;


import org.janelia.workstation.integration.activity_logging.ActionString;

import org.janelia.workstation.integration.activity_logging.CategoryString;

import org.janelia.workstation.integration.activity_logging.ToolString;

/**
 * Implement this to expose session-manager functionality to more-or-less
 * external plugins.  Helps plugins function without direct dependency
 * on the console app.
 *
 * @author fosterl
 */
public interface ActivityLogging {
    public static final String LOOKUP_PATH = "ActivityLogging/Location/Nodes";
            
    /**
     * Send an event described by the information given as parameters, to the
     * logging apparatus. Apply the criteria of:
     * 1. allow-to-log if more time was taken, than the lower threshold, or
     * 2. allow-to-log if the count of attempts for category==granularity.
     *
     * @param toolName the stakeholder tool, in this event.
     * @param category for namespacing.
     * @param action what happened.
     * @param timestamp when it happened.
     * @param elapsedMs how much time passed to carry this out?
     * @param thresholdMs beyond this time, force log issue.
     */
    void logToolEvent(final ToolString toolName, final CategoryString category, final ActionString action, final long timestamp, final double elapsedMs, final double thresholdMs);

    /**
     * Log a tool event, always.  No criteria will be checked.
     */
    void logToolEvent(ToolString toolName, CategoryString category, ActionString action);

    /**
     * Log-tool-event override, which includes elapsed/threshold comparison
     * values.  If the elapsed time (expected milliseconds) exceeds the
     * threshold, definitely log.  Also, will check number-of-issues against
     * a granularity map.  Only issue the message at a preset
     * granularity.
     *
     * @param elapsedMs
     * @param thresholdMs
     */
    void logToolEvent(ToolString toolName, CategoryString category, ActionString action, double elapsedMs, double thresholdMs);

    /**
     * Send an event described by the information given as parameters, to the
     * logging apparatus. Apply the criteria of:
     * 1. allow-to-log if more time was taken, than the lower threshold, or
     *
     * @param toolName the stakeholder tool, in this event.
     * @param category for namespacing.
     * @param action what happened.
     * @param timestamp when it happened.
     * @param elapsedMs how much time passed to carry this out?
     * @param thresholdMs beyond this time, force log issue.
     * @todo see about reusing code between this and non-threshold.
     */
    void logToolThresholdEvent(final ToolString toolName, final CategoryString category, final ActionString action, final long timestamp, final double elapsedMs, final double thresholdMs);

}
