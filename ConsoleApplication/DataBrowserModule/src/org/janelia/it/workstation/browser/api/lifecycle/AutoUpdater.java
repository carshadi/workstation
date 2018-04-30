package org.janelia.it.workstation.browser.api.lifecycle;

import java.net.URL;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;

import javax.swing.JOptionPane;

import org.janelia.it.jacs.integration.FrameworkImplProvider;
import org.janelia.it.workstation.browser.gui.support.WindowLocator;
import org.janelia.it.workstation.browser.util.SystemInfo;
import org.janelia.it.workstation.browser.workers.SimpleWorker;
import org.netbeans.api.autoupdate.InstallSupport;
import org.netbeans.api.autoupdate.InstallSupport.Installer;
import org.netbeans.api.autoupdate.InstallSupport.Validator;
import org.netbeans.api.autoupdate.OperationContainer;
import org.netbeans.api.autoupdate.OperationContainer.OperationInfo;
import org.netbeans.api.autoupdate.OperationException;
import org.netbeans.api.autoupdate.OperationSupport.Restarter;
import org.netbeans.api.autoupdate.UpdateElement;
import org.netbeans.api.autoupdate.UpdateManager;
import org.netbeans.api.autoupdate.UpdateUnit;
import org.netbeans.api.autoupdate.UpdateUnitProvider;
import org.netbeans.api.autoupdate.UpdateUnitProviderFactory;
import org.netbeans.api.progress.ProgressHandle;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Download updates automatically on startup and ask the user to restart, or install the next time 
 * the application is started.
 * 
 * Most of the update code comes from this guide: https://blogs.oracle.com/rechtacek/entry/how_to_update_netbeans_platform
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class AutoUpdater extends SimpleWorker {

    private static final Logger log = LoggerFactory.getLogger(AutoUpdater.class);

    private static final String UPDATE_CENTER_NAME = "org_janelia_it_workstation_nb_action_update_center";
    private static final String UPDATE_CENTER_LABEL = "Janelia Workstation Update Center";
    private static final String OLD_UPDATE_CENTER_URL = "http://jacs-webdav.int.janelia.org/workstation6/updates.xml";
    private static final String NEW_UPDATE_CENTER_URL = "http://jacs-webdav.int.janelia.org/workstation6/updates.xml";
    
    private OperationContainer<InstallSupport> containerForUpdate;
    private Restarter restarter;
    private boolean restarting = false;

    @Override
    protected void doStuff() throws Exception {
        
        upgradeToNewUpdateCenter();
        
        ProgressHandle handle = ProgressHandle.createHandle("Checking for updates...");
        
        try {
            handle.start();
            this.containerForUpdate = getContainerForUpdate(doRealCheck(handle));

            if (containerForUpdate.getSupport()==null) {
                log.info("Found no updates to install");
                return;
            }
            
            log.info("Checking licenses");
            if (!allLicensesApproved(containerForUpdate)) {
                log.warn("Licenses are not approved. Aborting update.");
                return;
            }
            
            log.info("Downloading updates...");
            handle.progress("Downloading updates...");
            Validator validator = doDownload(containerForUpdate);
            
            log.info("Installing updates...");
            handle.progress("Installing updates...");
            this.restarter = doInstall(containerForUpdate.getSupport(), validator);
        }
        finally {
            handle.finish();
        }
    }

    private void upgradeToNewUpdateCenter() {

        try {
            List<UpdateUnitProvider> updateUnitProviders = UpdateUnitProviderFactory.getDefault().getUpdateUnitProviders(true);
            for (UpdateUnitProvider provider : updateUnitProviders) {
                if (UPDATE_CENTER_LABEL.equals(provider.getDisplayName()) && OLD_UPDATE_CENTER_URL.equals(provider.getProviderURL())) {
                    log.warn("Removing legacy update center {} ({})", provider.getName(), provider.getProviderURL());
                    UpdateUnitProviderFactory.getDefault().remove(provider);
                    UpdateUnitProvider newProvider = UpdateUnitProviderFactory.getDefault().create(UPDATE_CENTER_NAME, UPDATE_CENTER_LABEL, new URL(NEW_UPDATE_CENTER_URL));
                    log.warn("Created update center {} ({})", newProvider.getName(), newProvider.getProviderURL());
                }
            }
        }
        catch (Exception ex) {
            log.error("Error updating to new update center", ex);
        }
    }

    @Override
    protected void hadSuccess() {
        if (containerForUpdate==null || restarter==null) return;
        InstallSupport support = containerForUpdate.getSupport();
        
        try {
            String html = "<html><body>"
                    + "<p>Updates have been downloaded are ready to install.</p>"
                    + "</body></html>";

            String[] buttons = { "Restart and Update", "Later" };
            int selectedOption = JOptionPane.showOptionDialog(WindowLocator.getMainFrame(), html, 
                    "Updates Ready", JOptionPane.INFORMATION_MESSAGE, 0, null, buttons, buttons[0]);

            if (selectedOption == 0) {
                log.info("Restarting now to complete installation.");
                support.doRestart(restarter, null);
                this.restarting = true;
            }
            else if (selectedOption == 1) {
                log.info("Will install updates the next time the application is started.");
                support.doRestartLater(restarter);
            }
        }
        catch (Throwable ex) {
            hadError(ex);
        }
    }

    @Override
    protected void hadError(Throwable ex) {
        FrameworkImplProvider.handleException("Error running auto-update check", ex);
    }
        
    private Collection<UpdateElement> doRealCheck(ProgressHandle handle) {    
        
        List<UpdateUnitProvider> updateUnitProviders = UpdateUnitProviderFactory.getDefault().getUpdateUnitProviders(true);
        for (UpdateUnitProvider provider : updateUnitProviders) {
            try {
                // the second parameter forces update from server when true
                log.info("Checking provider '{}'", provider.getDisplayName());
                handle.progress("Checking "+ provider.getDisplayName());
                provider.refresh(handle, true);
            }
            catch (Exception ex) {
                log.error("Error refreshing " + provider.getDisplayName(), ex);
            }
        }
        
        Collection<UpdateElement> elements4update = new HashSet<>();
        List<UpdateUnit> updateUnits = UpdateManager.getDefault().getUpdateUnits();
        for (UpdateUnit unit : updateUnits) {
            handle.progress("Finding updates for "+unit.getCodeName());
            // plugin already installed?
            if (unit.getInstalled() != null) {
                // has updates ?
                if (!unit.getAvailableUpdates().isEmpty()) {
                    log.info("Found updates for: {}", unit.getCodeName());
                    // add plugin with highest version
                    elements4update.add(unit.getAvailableUpdates().get(0));
                }
                else {
                    log.debug("No updates for: {}", unit.getCodeName());
                }
            }
        }
        return elements4update;
    }

    private OperationContainer<InstallSupport> getContainerForUpdate(Collection<UpdateElement> elements4update) {
        OperationContainer<InstallSupport> container = OperationContainer.createForUpdate();
        for (UpdateElement element : elements4update) {
            log.info("Checking update element: {}", element.getDisplayName());
            
            if (container.canBeAdded(element.getUpdateUnit(), element)) {
                log.info("Adding update element: {}", element.getDisplayName());
                
                OperationInfo<InstallSupport> operationInfo = container.add(element);
                if (operationInfo == null) {
                    continue;
                }

                if (!operationInfo.getBrokenDependencies().isEmpty()) {
                    log.info("Found broken dependencies for: {}", element.getDisplayName());
                    continue;
                }
                
                log.info("Adding required elements for: {}", element.getDisplayName());
                container.add(operationInfo.getRequiredElements());
            }
        }
        
        return container;
    }

    private boolean allLicensesApproved(OperationContainer<InstallSupport> container) {
        if (!container.listInvalid().isEmpty()) {
            return false;
        }
        for (OperationInfo<InstallSupport> info : container.listAll()) {
            String license = info.getUpdateElement().getLicence();
            if (!isLicenseApproved(license)) {
                return false;
            }
        }
        return true;
    }

    private boolean isLicenseApproved(String license) {
        // TODO: check this for real, and pop-up licenses if they need approval 
        // (this seems like something the framework should do for us...)
        return true;
    }

    private Validator doDownload(OperationContainer<InstallSupport> container) throws OperationException {
        InstallSupport installSupport = container.getSupport();
        
        // The following is unfortunate. The autoupdate Utilities class, which is the only way to access the user's install dir preference, 
        // is part of org.netbeans.modules.autoupdate.ui, which is not a public module API. We could use implementation version here, but it 
        // can lead to a lot of problems with auto update. For now, we'll assume global installation, and let the user do a manual upgrade if
        // things don't work.
        Boolean global = true;//Utilities.isGlobalInstallation();
        boolean userDirAsFallback = true;
        
        try {
            String installDir = SystemInfo.getInstallDir();
            log.info("Install directory: "+installDir);
            
            if (installDir.startsWith("/misc/local/workstation")) {
                log.warn("Shared Linux installation detected. Forcing global installation.");
                // if using the shared Linux installation disallow user dir upgrades so that all users stay in sync
                global = true;
                userDirAsFallback = false;
            }
        }
        catch (RuntimeException e) {
            // The above step isn't critical, so we handle any exceptions to allow install to continue. 
            FrameworkImplProvider.handleException(e);
        }
        
        log.info("Downloading updates with flags: global={}, userDirAsFallback={}", global, userDirAsFallback);
        return installSupport.doDownload(null, global, userDirAsFallback);
    }

    private Restarter doInstall(InstallSupport support, Validator validator) throws OperationException {
        // validates all plugins are correctly downloaded
        Installer installer = support.doValidate(validator, null);
        return support.doInstall(installer, null);
    }

    public boolean isRestarting() {
        return restarting;
    }
}
