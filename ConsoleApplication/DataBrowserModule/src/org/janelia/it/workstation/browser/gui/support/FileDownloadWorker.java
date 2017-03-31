package org.janelia.it.workstation.browser.gui.support;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.locks.Lock;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.janelia.it.jacs.model.tasks.Task;
import org.janelia.it.jacs.model.tasks.TaskMessage;
import org.janelia.it.jacs.model.tasks.TaskParameter;
import org.janelia.it.workstation.browser.ConsoleApp;
import org.janelia.it.workstation.browser.api.StateMgr;
import org.janelia.it.workstation.browser.util.Utils;
import org.janelia.it.workstation.browser.workers.BackgroundWorker;
import org.janelia.it.workstation.browser.workers.TaskMonitoringWorker;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Worker for downloading a sample image (or set of split channel images) in a specific format.
 * The specified extension is used to determine whether conversion is necessary and when necessary,
 * if conversion can be managed locally or remotely (via task submission).
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class FileDownloadWorker {

    private static final Logger log = LoggerFactory.getLogger(FileDownloadWorker.class);
    
    private final DownloadItem downloadItem;
    private final Lock copyFileLock;
    private final String objectName;
    private final String targetExtension;
    private final String sourceFilePath;
    private final File targetDir;

    public FileDownloadWorker(DownloadItem downloadItem, Lock copyFileLock) {
        this.downloadItem = downloadItem;
        this.copyFileLock = copyFileLock;
        this.objectName = downloadItem.getDomainObject().getName();
        this.targetExtension = downloadItem.getTargetExtension();
        this.sourceFilePath = downloadItem.getSourceFile();
        this.targetDir = downloadItem.getTargetFile().getParentFile();
    }

    public void startDownload() {

        log.debug("Starting download of {} to {}", downloadItem.getSourceExtension(), downloadItem.getTargetExtension());

        try {
            boolean convertOnServer = true;

            if (!downloadItem.isSplitChannels()) {
                if (sourceFilePath.endsWith(targetExtension)) {
                    // no conversion needed, simply transfer the file
                    convertOnServer = false;
                } 
                else if (Utils.EXTENSION_LSM.equals(targetExtension)) {
                    // Just need to convert bz2 to lsm, which we can do locally
                    convertOnServer = false;
                }
            }
            
            if (convertOnServer) {
                convertOnServer();
            } 
            else {
                transferAndConvertLocallyAsNeeded();
            }
        }
        catch (Exception e) {
            ConsoleApp.handleException(e);
        }
    }
    
    private void copyFile(String remoteFile,
                          File localFile,
                          BackgroundWorker worker) throws Exception {
        worker.setStatus("Waiting to download...");
        copyFileLock.lock();
        try {
            worker.setStatus("Downloading " + new File(remoteFile).getName());
            Utils.copyURLToFile(remoteFile, localFile, worker);
        } 
        finally {
            copyFileLock.unlock();
        }
    }

    private void convertOnServer() throws Exception {

        log.info("Converting {} to {} (splitChannels={})",sourceFilePath,targetExtension,downloadItem.isSplitChannels());
        
        Task task;
        if (downloadItem.isSplitChannels()) {
            HashSet<TaskParameter> taskParameters = new HashSet<>();
            taskParameters.add(new TaskParameter("filepath", sourceFilePath, null));
            taskParameters.add(new TaskParameter("output extension", targetExtension, null));
            task = StateMgr.getStateMgr().submitJob("ConsoleSplitChannels", "Split Channels: "+objectName, taskParameters);
        }
        else {
            HashSet<TaskParameter> taskParameters = new HashSet<>();
            taskParameters.add(new TaskParameter("filepath", sourceFilePath, null));
            taskParameters.add(new TaskParameter("output extension", targetExtension, null));
            task = StateMgr.getStateMgr().submitJob("ConsoleConvertFile", "Convert: "+objectName, taskParameters);
        }

        TaskMonitoringWorker taskWorker = new TaskMonitoringWorker(task.getObjectId()) {

            @Override
            public String getName() {
                return "Downloading " + objectName;
            }

            @Override
            protected void doStuff() throws Exception {

                setStatus("Grid execution");

                // Wait until task is finished
                super.doStuff();

                throwExceptionIfCancelled();

                setStatus("Parse result");

                // Since there is no way to log task output vars, we use a convention where the last message
                // will contain the output files.
                String resultFiles = null;
                final Task task = getTask();
                List<TaskMessage> messages = new ArrayList<>(task.getMessages());
                if (! messages.isEmpty()) {
                    Collections.sort(messages, new Comparator<TaskMessage>() {
                        @Override
                        public int compare(TaskMessage o1, TaskMessage o2) {
                            return o2.getMessageId().compareTo(o1.getMessageId());
                        }
                    });
                    resultFiles = messages.get(0).getMessage();
                }

                throwExceptionIfCancelled();

                if (resultFiles==null) {
                    throw new Exception("No result files generated");
                }

                // Copy the files to the local drive
                String[] pathAndFiles = resultFiles.split(":");
                String path = pathAndFiles[0];
                String[] filePaths = pathAndFiles[1].split(",");
                for(String filePath : filePaths) {
                    String remoteFile = path + "/" + filePath;
                    String targetFile = downloadItem.getTargetFile().getAbsolutePath();
                    
                    String channelSuffix = getChannelSuffix(filePath);
                    if (channelSuffix!=null) {
                        targetFile = targetFile.replaceAll("#",channelSuffix);    
                    }
                    
                    File localFile = new File(targetFile);
                    copyFile(remoteFile, localFile, this);
                }

                throwExceptionIfCancelled();
                setFinalStatus("Done");
            }

            @Override
            public Callable<Void> getSuccessCallback() {
                return getDownloadSuccessCallback();
            }
        };

        taskWorker.executeWithEvents();
    }

    private String getChannelSuffix(String filepath) {
        Pattern p = Pattern.compile("(.*)(_(c\\d))(.*)");
        Matcher m = p.matcher(filepath);
        if (m.matches()) {
            return m.group(3);
        }
        return null;
    }
    
    private void transferAndConvertLocallyAsNeeded() {

        final String remoteFile = downloadItem.getSourceFile();
        final File localFile = downloadItem.getTargetFile();

        log.info("Transferring {} to {}",remoteFile,localFile);
        
        final BackgroundWorker transferWorker = new BackgroundWorker() {

            @Override
            public String getName() {
                return "Downloading " + new File(remoteFile).getName();
            }

            @Override
            protected void doStuff() throws Exception {
                setStatus("Local execution");
                throwExceptionIfCancelled();
                copyFile(remoteFile, localFile, this);
                throwExceptionIfCancelled();
                setFinalStatus("Done");
            }

            @Override
            public Callable<Void> getSuccessCallback() {
                return getDownloadSuccessCallback();
            }
        };

        transferWorker.executeWithEvents();
    }

    private Callable<Void> getDownloadSuccessCallback() {
        return new Callable<Void>() {
            @Override
            public Void call() throws Exception {
                DesktopApi.browse(targetDir);
                return null;
            }
        };
    }

    public DownloadItem getDownloadItem() {
        return downloadItem;
    }

    public String getObjectName() {
        return objectName;
    }

    public String getTargetExtension() {
        return targetExtension;
    }

    public String getSourceFilePath() {
        return sourceFilePath;
    }

    public File getTargetDir() {
        return targetDir;
    }
}