package org.janelia.it.workstation.browser.logging;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Iterator;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;
import java.util.logging.Handler;
import java.util.logging.LogRecord;

import javax.swing.JButton;
import javax.swing.JOptionPane;
import javax.swing.SwingUtilities;

import org.janelia.it.jacs.shared.utils.StringUtils;
import org.janelia.it.workstation.browser.ConsoleApp;
import org.janelia.it.workstation.browser.api.AccessManager;
import org.janelia.it.workstation.browser.gui.support.MailDialogueBox;
import org.janelia.it.workstation.browser.util.ConsoleProperties;
import org.janelia.it.workstation.browser.util.SystemInfo;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.base.Charsets;
import com.google.common.collect.ConcurrentHashMultiset;
import com.google.common.collect.Multiset;
import com.google.common.hash.HashFunction;
import com.google.common.hash.Hashing;
import com.google.common.util.concurrent.RateLimiter;

/**
 * Override NetBeans' exception handling to tie into the workstation's error handler.
 *
 * When the application starts up, this exception handler must be registered like this:
 *   java.util.logging.Logger.getLogger("").addHandler(new NBExceptionHandler());
 *
 * This is implemented according to the advice given on the official NetBeans documentation, here:
 * http://wiki.netbeans.org/DevFaqCustomizingUnexpectedExceptionDialog
 * 
 * However, it has some major flaws, mainly related to the fact that the Throwable being handled is the last 
 * published Throwable, not the Throwable that the user has selected on using the Prev/Next buttons. If 
 * an exception happens asynchronously after the dialog shows up, then the throwable will be updated as well.
 * In general it's a very shoddy solution, but it's currently the best we can do without rewriting all the 
 * NetBeans error handling. 
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class NBExceptionHandler extends Handler implements Callable<JButton>, ActionListener {

    private static final Logger log = LoggerFactory.getLogger(NBExceptionHandler.class);

    private static final int COOLDOWN_TIME_SEC = 300; // Allow one auto exception report every 5 minutes
    private static final int MAX_STACKTRACE_CACHE_SIZE = 1000; // Keep track of a max number of unique stacktraces
    private static final boolean AUTO_SEND_EXCEPTIONS = ConsoleProperties.getBoolean("console.AutoSendExceptions", false);
    private static final String APP_VERSION = ConsoleApp.getConsoleApp().getApplicationVersion();

    private final HashFunction hf = Hashing.md5();
    
    private boolean notified = false;
    
    private Throwable throwable;
    private JButton newFunctionButton;

    private final Multiset<String> exceptionCounts = ConcurrentHashMultiset.create();
    private final RateLimiter rateLimiter = RateLimiter.create(1);
    
    @Override
    public void publish(LogRecord record) {
        if (record.getThrown()!=null) {
            this.throwable = record.getThrown();
            if (AccessManager.getAccessManager().isLoggedIn()) { // JW-25430: Only attempt to auto-send exceptions once the user has logged in
                try {
                    autoSendNovelExceptions();
                }
                catch (Throwable t) {
                    log.error("Error attempting to auto-send exceptions", t);
                }
            }
        }
    }
    
    /**
     * If an exception has not appeared before during this session, go ahead and create a JIRA ticket for it. 
     */
    private synchronized void autoSendNovelExceptions() {

        if ("DEV".equals(APP_VERSION)) {
            return;
        }
        
        if (!AUTO_SEND_EXCEPTIONS) {
            if (!notified) {
                notified = true;
                log.warn("Auto-sending exceptions is not configured. To configure auto-send, set console.AutoSendExceptions=true in console.properties.");
            }
            return;
        }

        String st = getStacktrace(throwable);
        String sth = hf.newHasher().putString(st, Charsets.UTF_8).hash().toString();
        log.trace("Got exception hash: {}",sth);
        String firstLine = getFirstLine(st);
        
        if (!exceptionCounts.contains(sth)) {
            // First appearance of this stack trace, let's try to send it to JIRA.

            // Allow one exception report every cooldown cycle. Our RateLimiter allows one access every 
            // second, so we need to acquire a cooldown's worth of locks.
            if (!rateLimiter.tryAcquire(COOLDOWN_TIME_SEC, 0, TimeUnit.SECONDS)) {
                log.warn("Exception reports exceeded email rate limit. Omitting auto-send of: {}", firstLine);
                return;
            }
            sendEmail(st, false);
        }
        else {
            int count = exceptionCounts.count(sth);
            if (count % 10 == 0) {
                log.warn("Exception count reached {} for: {}", count, firstLine);
                // TODO: create another JIRA ticket?
            }
        }

        exceptionCounts.add(sth); // Increment counter

        // Make sure we're not devoting too much memory to stack traces
        if (exceptionCounts.size()>MAX_STACKTRACE_CACHE_SIZE) {
            log.info("Purging single exceptions (cacheSize={})", exceptionCounts.size());
            // Try purging all singleton exceptions.
            for (Iterator<String> i = exceptionCounts.iterator(); i.hasNext();) {
                if (exceptionCounts.count(i.next())<2) {
                    i.remove();
                }
            }
            // Still too big? Just clear it out.
            if (exceptionCounts.size()>=MAX_STACKTRACE_CACHE_SIZE) {
                log.info("Clearing exception cache (cacheSize={})", exceptionCounts.size());
                exceptionCounts.clear();
            }
        }   
    }

    @Override
    public void flush() {
    }

    @Override
    public void close() throws SecurityException {
        this.throwable = null;
    }

    // Return the button we want to be displayed in the Uncaught Exception Dialog.
    @Override
    public JButton call() throws Exception {
        if (newFunctionButton==null) {
            newFunctionButton = new JButton("Report This Issue");
            newFunctionButton.addActionListener(this);
        }
        return newFunctionButton;
    }
    
    @Override
    public void actionPerformed(ActionEvent e) {
        SwingUtilities.windowForComponent(newFunctionButton).setVisible(false);
        // Due to the way the NotifyExcPanel works, this might not be the exception the user is currently looking at! 
        // Maybe it's better than nothing if it's right 80% of the time? 
        sendEmail(getStacktrace(throwable), true);
    }
    
    private void sendEmail(String stacktrace, boolean askForInput) {

        try {
            String firstLine = getFirstLine(stacktrace);
            log.info("Reporting exception: "+firstLine);
            
            String titleSuffix = " from "+AccessManager.getUsername()+" -- "+firstLine;

            MailDialogueBox mailDialogueBox = new MailDialogueBox(ConsoleApp.getMainFrame(),
                    (String) ConsoleApp.getConsoleApp().getModelProperty(AccessManager.USER_EMAIL),
                    (askForInput?"User-reported Exception":"Auto-reported Exception")+titleSuffix,
                    askForInput?"Problem Description: ":"");
            try {
                StringBuilder sb = new StringBuilder();
                sb.append("\nSubject Key: ").append(AccessManager.getSubjectKey());
                sb.append("\nApplication: ").append(ConsoleApp.getConsoleApp().getApplicationName()).append(" v").append(ConsoleApp.getConsoleApp().getApplicationVersion());
                sb.append("\nOperating System: ").append(SystemInfo.getOSInfo());
                sb.append("\nJava: ").append(SystemInfo.getJavaInfo());
                sb.append("\nRuntime: ").append(SystemInfo.getRuntimeJavaInfo());
                if (stacktrace!=null) {
                    sb.append("\n\nStack Trace:\n");
                    sb.append(stacktrace);
                }
                
                mailDialogueBox.addMessageSuffix(sb.toString());
            }
            catch (Exception e) {
                // Do nothing if the notification attempt fails.
                log.warn("Error building exception suffix" , e);
            }
            
            if (askForInput) {
                mailDialogueBox.showPopupThenSendEmail();
            }
            else {
                mailDialogueBox.sendEmail();
            }
        }
        catch (Exception ex) {
            log.warn("Error sending exception email",ex);
            if (askForInput) { // JW-25430: Only show this message if the email was initiated by the user
                JOptionPane.showMessageDialog(ConsoleApp.getMainFrame(), 
                        "Your message was NOT able to be sent to our support staff.  "
                        + "Please contact your support representative.", "Error sending email", JOptionPane.ERROR_MESSAGE);
            }
        }
    }
    
    private String getStacktrace(Throwable t) {
        StringBuilder sb = new StringBuilder();
        sb.append(t.getClass().getName()).append(": "+t.getMessage()).append("\n");
        int stackLimit = 100;
        int i = 0;
        for (StackTraceElement element : t.getStackTrace()) {
            if (element==null) continue;
            String s = element.toString();
            if (!StringUtils.isEmpty(s)) {
                sb.append("at ");
                sb.append(element.toString());
                sb.append("\n");
                if (i++>stackLimit) {
                    break;
                }
            }
        }
        return sb.toString();
    }
    
    private String getFirstLine(String st) {
        int n = st.indexOf('\n');
        if (n<1) return st; 
        return st.substring(0, n);
    }
}
