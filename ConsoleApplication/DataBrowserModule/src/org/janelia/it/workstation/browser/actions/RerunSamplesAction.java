package org.janelia.it.workstation.browser.actions;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.List;

import javax.swing.AbstractAction;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JTextField;

import org.janelia.it.workstation.browser.ConsoleApp;
import org.janelia.it.workstation.browser.activity_logging.ActivityLogHelper;
import org.janelia.it.workstation.browser.api.AccessManager;
import org.janelia.it.workstation.browser.api.ClientDomainUtils;
import org.janelia.it.workstation.browser.api.DomainMgr;
import org.janelia.it.workstation.browser.api.DomainModel;
import org.janelia.it.workstation.browser.components.DomainExplorerTopComponent;
import org.janelia.it.workstation.browser.gui.dialogs.ModalDialog;
import org.janelia.it.workstation.browser.gui.support.GroupedKeyValuePanel;
import org.janelia.it.workstation.browser.workers.SimpleWorker;
import org.janelia.model.access.domain.DomainUtils;
import org.janelia.model.domain.DomainObject;
import org.janelia.model.domain.dto.SampleReprocessingRequest;
import org.janelia.model.domain.enums.PipelineStatus;
import org.janelia.model.domain.enums.SubjectRole;
import org.janelia.model.domain.sample.Sample;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Created by fosterl on 8/15/2016.
 */
public class RerunSamplesAction extends AbstractAction {

    private static final Logger log = LoggerFactory.getLogger(RerunSamplesAction.class);
    
    private static final int MAX_SAMPLE_RERUN_COUNT = 10;
    private final List<Sample> samples;

    /**
     * Returns action or null.  Action will be returned, if the selected objects contain one or more samples, the
     * user is empowered to write those samples, and there are 10 or fewer samples.
     *
     * @param selectedObjects containing 1-10 samples.
     * @return named action or null.
     */
    public static RerunSamplesAction createAction(List<DomainObject> selectedObjects) {
        RerunSamplesAction action = null;
        List<Sample> samples = new ArrayList<>();
        for (DomainObject re : selectedObjects) {
            if (re == null) {
                log.info("Null object in selection.");
                continue;
            }
            if (re instanceof Sample) {
                
                Sample sample = (Sample)re;
                if (sample.getStatus() == null) {
                    log.info("Null sample status in selection Name={}, ID={}.", sample.getName(), sample.getId());
                }
                
                boolean canWrite = ClientDomainUtils.hasWriteAccess(sample);
                boolean canRerun = (!PipelineStatus.Processing.toString().equals(sample.getStatus())  &&
                        !PipelineStatus.Scheduled.toString().equals(sample.getStatus()));
                
                if ((canWrite && canRerun) || AccessManager.getAccessManager().isAdmin()) {
                    samples.add(sample);
                }
            }
        }
        if (!samples.isEmpty()) {
            action = new RerunSamplesAction(samples);
        }
        return action;
    }

    /**
     * Construct with everything needed to re-run.  C'tor is private because this is intended
     * to be run only under certain criteria.
     *
     * @param samples what to re-run.
     */
    private RerunSamplesAction(List<Sample> samples) {
        super(getName(samples));
        this.samples = samples;
    }

    public static final String getName(List<Sample> samples) {
        final String samplesText = (samples.size() > 1)?samples.size()+" Samples":"Sample";
        return ("Mark "+samplesText+" for Reprocessing");
    }
    
    @Override
    public void actionPerformed(ActionEvent event) {
        
        if (samples.size() > MAX_SAMPLE_RERUN_COUNT && !AccessManager.authenticatedSubjectIsInGroup(SubjectRole.Admin)) {
            JOptionPane.showMessageDialog(ConsoleApp.getMainFrame(), 
                    "You cannot submit more than "+MAX_SAMPLE_RERUN_COUNT+" samples for reprocessing at a time.",
                    "Too many samples selected", 
                    JOptionPane.ERROR_MESSAGE);
            return;
        }
        
        int numBlocked = 0;
        for (Sample sample : samples) {
            if (sample.isSampleBlocked()) {
                numBlocked++;
            }
        }
        
        StringBuilder sampleText = new StringBuilder();
        if (samples.size() == 1) {
            sampleText.append("sample");
        }
        else {
            sampleText.append(samples.size());
            sampleText.append(" samples");
        }
        
        ReprocessingDialog dialog = new ReprocessingDialog("Reprocess "+sampleText);
        if (!dialog.showDialog()) return;
        
        if (numBlocked>0) {
            int result2 = JOptionPane.showConfirmDialog(ConsoleApp.getMainFrame(), "You have selected "+numBlocked+" blocked samples for reprocessing. Continue with unblocking and reprocessing?",
                    "Blocked Samples Selected", JOptionPane.OK_CANCEL_OPTION);
            if (result2 != 0) return;
        }
        
        SimpleWorker sw = new SimpleWorker() {

            @Override
            protected void doStuff() throws Exception {
                for (Sample sample : samples) {
                    ActivityLogHelper.logUserAction("DomainObjectContentMenu.markForReprocessing", sample);    
                }
                DomainModel model = DomainMgr.getDomainMgr().getModel();
                
                SampleReprocessingRequest request = new SampleReprocessingRequest();
                request.setSampleReferences(DomainUtils.getReferences(samples));
                request.setProcessLabel("User Requested Reprocessing");
                request.setReuseSummary(dialog.isReuseSummary());
                request.setReuseProcessing(dialog.isReuseProcessing());
                request.setReusePost(dialog.isReusePost());
                request.setReuseAlignment(dialog.isReuseAlignment());
                request.setReuseSeparation(dialog.isReuseSeparation());
                request.setKeepExistingResults(dialog.isKeepExistingResults());
                request.setExtraOptions(dialog.getExtraOptions());
                
                model.dispatchSamples(request);
            }

            @Override
            protected void hadSuccess() {
                log.debug("Successfully dispatched "+samples.size()+" samples.");
                DomainExplorerTopComponent.getInstance().refresh(true, true, null);
            }

            @Override
            protected void hadError(Throwable error) {
                ConsoleApp.handleException(error);
            }
            
        };
        sw.execute();
    }
    

    private class ReprocessingDialog extends ModalDialog {

        private final GroupedKeyValuePanel mainPanel;
        private final JCheckBox reuseSummaryCheckbox;
        private final JCheckBox reuseProcessingCheckbox; 
        private final JCheckBox reusePostCheckbox; 
        private final JCheckBox reuseAlignmentCheckbox;
        private final JCheckBox reuseSeparationCheckbox;
        private final JCheckBox keepResultsCheckbox;
        private final JTextField extraOptionsField;
        
        private boolean returnValue;
        
        public ReprocessingDialog(String okButtonName) {
            
            setTitle("Reprocess Samples");
            setLayout(new BorderLayout());

            this.mainPanel = new GroupedKeyValuePanel();
            mainPanel.addSeparator("Reuse existing results");
            
            this.reuseSummaryCheckbox = new JCheckBox();
            mainPanel.addItem("LSM Summary", reuseSummaryCheckbox);
            
            this.reuseProcessingCheckbox = new JCheckBox();
            mainPanel.addItem("Sample Processing", reuseProcessingCheckbox);
            
            this.reusePostCheckbox = new JCheckBox();
            mainPanel.addItem("Post-Processing", reusePostCheckbox);
            
            this.reuseAlignmentCheckbox = new JCheckBox();
            mainPanel.addItem("Alignment", reuseAlignmentCheckbox);

            this.reuseSeparationCheckbox = new JCheckBox();
            mainPanel.addItem("Separation", reuseSeparationCheckbox);
            
            mainPanel.addSeparator("Other options");
            this.keepResultsCheckbox = new JCheckBox();
            mainPanel.addItem("Keep previous results", keepResultsCheckbox);
            
            this.extraOptionsField = new JTextField();
            extraOptionsField.setColumns(20);
            if (AccessManager.getAccessManager().isAdmin()) {
                mainPanel.addItem("Extra options", extraOptionsField);
            }
            
            add(mainPanel, BorderLayout.CENTER);
            
            JButton okButton = new JButton(okButtonName);
            okButton.setToolTipText("Schedule selected samples for reprocessing");
            okButton.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    returnValue = true;
                    setVisible(false);
                }
            });

            JButton cancelButton = new JButton("Cancel");
            cancelButton.setToolTipText("Cancel without reprocessing");
            cancelButton.addActionListener(new ActionListener() {
                @Override
                public void actionPerformed(ActionEvent e) {
                    returnValue = false;
                    setVisible(false);
                }
            });

            JPanel buttonPane = new JPanel();
            buttonPane.setLayout(new BoxLayout(buttonPane, BoxLayout.LINE_AXIS));
            buttonPane.setBorder(BorderFactory.createEmptyBorder(20, 10, 10, 10));
            buttonPane.add(Box.createHorizontalGlue());
            buttonPane.add(okButton);
            buttonPane.add(Box.createRigidArea(new Dimension(10, 0)));
            buttonPane.add(cancelButton);
            add(buttonPane, BorderLayout.SOUTH);

            getRootPane().setDefaultButton(okButton);
            
            addWindowListener(new WindowAdapter() {
                public void windowClosing(WindowEvent e) {
                    returnValue = false;
                }
            });
        }
        
        public boolean showDialog() {
            packAndShow();
            // Blocks until dialog is no longer visible, and then:
            removeAll();
            dispose();
            log.info("Returning value: "+returnValue);
            return returnValue;
        }


        public boolean isReuseSummary() {
            return reuseSummaryCheckbox.isSelected();
        }


        public boolean isReuseProcessing() {
            return reuseProcessingCheckbox.isSelected();
        }


        public boolean isReusePost() {
            return reusePostCheckbox.isSelected();
        }


        public boolean isReuseAlignment() {
            return reuseAlignmentCheckbox.isSelected();
        }
        
        public boolean isReuseSeparation() {
            return reuseSeparationCheckbox.isSelected();
        }

        public boolean isKeepExistingResults() {
            return keepResultsCheckbox.isSelected();
        }


        public String getExtraOptions() {
            return extraOptionsField.getText();
        }
    }
}
