package org.janelia.it.workstation.browser.gui.support;

import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextArea;

import org.janelia.it.jacs.shared.utils.MailHelper;
import org.janelia.it.workstation.browser.util.ConsoleProperties;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created with IntelliJ IDEA.
 * User: kimmelr
 * Date: 5/22/12
 * Time: 2:51 PM
 */
public class MailDialogueBox {

    private Logger log = LoggerFactory.getLogger(MailDialogueBox.class);
    
    private final String toEmail = ConsoleProperties.getString("console.HelpEmail");
    private String fromEmail;
    private String subject = "";
    private String messagePrefix = "";
    private String messageSuffix = "";
    private JFrame parentFrame;

    public MailDialogueBox(JFrame parentFrame, String fromEmail, String subject) {
        this.fromEmail = fromEmail;
        this.subject = subject;
        this.parentFrame = parentFrame;
    }

    public MailDialogueBox(JFrame parentFrame, String fromEmail, String subject, String messagePrefix){
        this.fromEmail = fromEmail;
        this.subject = subject;
        this.parentFrame = parentFrame;
        this.messagePrefix = messagePrefix;
    }

    public void showPopupThenSendEmail(){
        String desc = null;
        JPanel panel = new JPanel();
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        panel.add(new JLabel("Please give a quick description of the problem and any other useful information."));
        panel.add(Box.createVerticalStrut(15));
        JTextArea textArea = new JTextArea(4, 20);
        panel.add(new JScrollPane(textArea));
        int ans;
        while (desc == null || desc.equals("")) {
            ans = JOptionPane.showConfirmDialog(parentFrame, panel, "Problem Description", JOptionPane.OK_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE);
            if (ans == JOptionPane.CANCEL_OPTION) return;
            desc = messagePrefix + textArea.getText() +"\n";
        }
        desc+=messageSuffix;
        MailHelper helper = new MailHelper();
        log.info("Sending email from "+fromEmail+" to "+toEmail);
        helper.sendEmail(fromEmail, toEmail, subject, desc);
    }

//    private JOptionPane getOptionPane() {
//        JFrame mainFrame = new JFrame();
//        if (parentFrame != null) mainFrame.setIconImage(parentFrame.getIconImage());
//        JOptionPane optionPane = new JOptionPane();
//        mainFrame.getContentPane().add(optionPane);
//        return optionPane;
//    }

    /**
     * This method gives the caller a way to place text at the end of the message.
     * @param messageSuffixInformation Text to display after everything else.
     */
    public void addMessageSuffix(String messageSuffixInformation) {
        this.messageSuffix = messageSuffixInformation;
    }
}