package org.janelia.it.workstation.gui.large_volume_viewer.action;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

import javax.swing.AbstractAction;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;

import org.janelia.it.jacs.model.user_data.tiledMicroscope.TmNeuron;
import org.janelia.it.workstation.gui.large_volume_viewer.annotation.AnnotationModel;
import org.janelia.it.workstation.shared.workers.SimpleWorker;

/**
 * this action opens a dialog that allows the user to edit
 * the tags on a neuron
 */
public class NeuronTagsAction extends AbstractAction {

    private AnnotationModel annModel;
    private TmNeuron currentNeuron;

    private JPanel mainPanel;
    private JList appliedTagsList;
    private JList availableTagsList;
    private JTextField newTagField;

    public NeuronTagsAction(AnnotationModel annotationModel) {
        this.annModel = annotationModel;

        putValue(NAME, "Edit neuron tags...");
        putValue(SHORT_DESCRIPTION, "Edit tags for neuron");
    }

    @Override
    public void actionPerformed(ActionEvent action) {

        // check for selected neuron
        currentNeuron = annModel.getCurrentNeuron();
        if (currentNeuron == null) {
            return;
        }

        Object[] options = {"Close"};
        JOptionPane.showOptionDialog(null,
            getInterface(),
            "Edit tags for " + currentNeuron.getName(),
            JOptionPane.DEFAULT_OPTION,
            JOptionPane.PLAIN_MESSAGE,
            null,
            options,
            options[0]
            );
    }

    /**
     * add given tag to current neuron
     */
    private void addTag(final String tag) {
        SimpleWorker adder = new SimpleWorker() {
            @Override
            protected void doStuff() throws Exception {
                annModel.addNeuronTag(tag, annModel.getCurrentNeuron());
            }

            @Override
            protected void hadSuccess() {
                fillTagLists();
            }

            @Override
            protected void hadError(Throwable error) {
                showError("There was an error adding the " + tag + " tag!", "Error");
            }
        };
        adder.execute();
    }

    /**
     * remove given tag from current neuron
     */
    private void removeTag(final String tag) {
        SimpleWorker remover = new SimpleWorker() {
            @Override
            protected void doStuff() throws Exception {
                annModel.removeNeuronTag(tag, annModel.getCurrentNeuron());
            }

            @Override
            protected void hadSuccess() {
                fillTagLists();
            }

            @Override
            protected void hadError(Throwable error) {
                showError("There was an error removing the " + tag + " tag!", "Error");
            }
        };
        remover.execute();
    }

    /**
     * create a new tag and apply it to the current neuron
     */
    private void onAddNewTag() {
        addTag(newTagField.getText());
    }

    /**
     * remove all the tags from the currently selected neuron
     */
    private void onRemoveAllButton() {
        SimpleWorker remover = new SimpleWorker() {
            @Override
            protected void doStuff() throws Exception {
                annModel.clearNeuronTags(annModel.getCurrentNeuron());
            }

            @Override
            protected void hadSuccess() {
                fillTagLists();
            }

            @Override
            protected void hadError(Throwable error) {
                showError("There was an error removing the tags!", "Error");
            }
        };
        remover.execute();
    }

    /**
     * populate both of the tag lists in the UI
     */
    private void fillTagLists() {
        // note: I've learned that people like things to be in order;
        //  so even though tags are inherently unordered, alphabetize
        //  them in the lists
        // note: it's lexical sort, though, so it might need adjustment
        //  if people complain

        // OMG Java makes everything hard...JList can't take List<>, only
        //  arrays and vectors
        Set<String> appliedTagSet = annModel.getNeuronTags(currentNeuron);
        String[] appliedTags = appliedTagSet.toArray(new String[appliedTagSet.size()]);
        Arrays.sort(appliedTags);
        appliedTagsList.setListData(appliedTags);

        // available tag list; be careful, we're mutating:
        Set<String> availableTagSet = new HashSet<>(annModel.getAllNeuronTags());
        availableTagSet.removeAll(appliedTagSet);
        String [] availableTags = availableTagSet.toArray(new String[availableTagSet.size()]);
        Arrays.sort(availableTags);
        availableTagsList.setListData(availableTags);
    }

    private void showError(String message, String title) {
        JOptionPane.showMessageDialog(
                mainPanel,
                message,
                title,
                JOptionPane.ERROR_MESSAGE);
    }

    private JPanel getInterface() {

        mainPanel = new JPanel();
        mainPanel.setLayout(new GridBagLayout());

        // two columns; applied tags on the left, other available
        //  tags on the right

        // I hate GridBagLayout's verbosity, but it's the only way
        //  in Java to get the control over the UI that I want

        Insets insets = new Insets(5, 5, 5, 5);

        // left column = applied tags
        GridBagConstraints cTopLeft = new GridBagConstraints();
        cTopLeft.gridx = 0;
        cTopLeft.gridy = 0;
        cTopLeft.anchor = GridBagConstraints.PAGE_START;
        cTopLeft.fill = GridBagConstraints.HORIZONTAL;
        cTopLeft.weighty = 0.0;
        cTopLeft.insets = insets;
        JLabel appliedLabel = new JLabel("Applied (click to remove)");
        appliedLabel.setHorizontalAlignment(JLabel.CENTER);
        mainPanel.add(appliedLabel, cTopLeft);

        // the dialog isn't currently resizeable, but I want
        //  to be ready in case I make it so
        GridBagConstraints cGrowLeft = new GridBagConstraints();
        cGrowLeft.gridx = 0;
        cGrowLeft.gridy = GridBagConstraints.RELATIVE;
        cGrowLeft.anchor = GridBagConstraints.PAGE_START;
        cGrowLeft.fill = GridBagConstraints.HORIZONTAL;
        cGrowLeft.weighty = 1.0;
        cGrowLeft.insets = insets;
        appliedTagsList = new JList();
        JScrollPane appliedScroller = new JScrollPane(appliedTagsList);
        appliedScroller.setPreferredSize(new Dimension(160, 160));
        mainPanel.add(appliedScroller, cGrowLeft);

        GridBagConstraints cFlowLeft = new GridBagConstraints();
        cFlowLeft.gridx = 0;
        cFlowLeft.gridy = GridBagConstraints.RELATIVE;
        cFlowLeft.anchor = GridBagConstraints.PAGE_START;
        cFlowLeft.fill = GridBagConstraints.NONE;
        cFlowLeft.weighty = 0.0;
        cFlowLeft.insets = insets;
        JButton removeAllButton = new JButton("Remove all");
        removeAllButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                onRemoveAllButton();
            }
        });
        mainPanel.add(removeAllButton, cFlowLeft);


        // right column = available tags
        GridBagConstraints cTopRight = new GridBagConstraints();
        cTopRight.gridx = 1;
        cTopRight.gridy = 0;
        cTopRight.anchor = GridBagConstraints.PAGE_START;
        cTopRight.fill = GridBagConstraints.HORIZONTAL;
        cTopRight.weighty = 0.0;
        cTopRight.insets = insets;
        JLabel availableLabel = new JLabel("Available (click to add)");
        availableLabel.setHorizontalAlignment(JLabel.CENTER);
        mainPanel.add(availableLabel, cTopRight);

        GridBagConstraints cGrowRight = new GridBagConstraints();
        cGrowRight.gridx = 1;
        cGrowRight.gridy = GridBagConstraints.RELATIVE;
        cGrowRight.anchor = GridBagConstraints.PAGE_START;
        cGrowRight.fill = GridBagConstraints.HORIZONTAL;
        cGrowRight.weighty = 1.0;
        cGrowRight.insets = insets;
        availableTagsList = new JList();
        JScrollPane availableScroller = new JScrollPane(availableTagsList);
        availableScroller.setPreferredSize(new Dimension(160, 160));
        mainPanel.add(availableScroller, cGrowRight);

        GridBagConstraints cFlowRight = new GridBagConstraints();
        cFlowRight.gridx = 1;
        cFlowRight.gridy = GridBagConstraints.RELATIVE;
        cFlowRight.anchor = GridBagConstraints.PAGE_START;
        cFlowRight.fill = GridBagConstraints.HORIZONTAL;
        cFlowRight.weighty = 0.0;
        cFlowRight.insets = insets;
        JPanel newTagPanel = new JPanel();
        newTagPanel.setLayout(new BoxLayout(newTagPanel, BoxLayout.LINE_AXIS));
        newTagField = new JTextField();
        newTagPanel.add(newTagField);
        JButton newTagButton = new JButton("New tag");
        newTagButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                onAddNewTag();
            }
        });
        newTagPanel.add(newTagButton);
        mainPanel.add(newTagPanel, cFlowRight);

        // new tag field behavior
        newTagField.addActionListener(new AbstractAction() {
            @Override
            public void actionPerformed(ActionEvent e) {
                onAddNewTag();
            }
        });

        // applied tags list behavior
        appliedTagsList.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                if (e.getClickCount() == 1) {
                    int index = appliedTagsList.locationToIndex(e.getPoint());
                    removeTag((String) appliedTagsList.getModel().getElementAt(index));
                }
            }
        });

        // available tags list behavior
        availableTagsList.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseClicked(MouseEvent e) {
                if (e.getClickCount() == 1) {
                    int index = availableTagsList.locationToIndex(e.getPoint());
                    addTag((String) availableTagsList.getModel().getElementAt(index));
                }
            }
        });

        fillTagLists();

        return mainPanel;
    }

}
