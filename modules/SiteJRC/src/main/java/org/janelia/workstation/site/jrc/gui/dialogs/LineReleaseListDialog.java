package org.janelia.workstation.site.jrc.gui.dialogs;

import org.janelia.model.domain.sample.LineRelease;
import org.janelia.workstation.common.gui.dialogs.ModalDialog;
import org.janelia.workstation.common.gui.support.Icons;
import org.janelia.workstation.common.gui.table.DynamicColumn;
import org.janelia.workstation.common.gui.table.DynamicRow;
import org.janelia.workstation.common.gui.table.DynamicTable;
import org.janelia.workstation.common.gui.util.UIUtils;
import org.janelia.workstation.core.activity_logging.ActivityLogHelper;
import org.janelia.workstation.core.api.DomainMgr;
import org.janelia.workstation.core.workers.SimpleWorker;
import org.janelia.workstation.integration.util.FrameworkAccess;

import javax.swing.*;
import java.awt.*;
import java.awt.event.MouseEvent;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.List;

/**
 * A dialog for viewing all the fly line releases that a user has access to.
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class LineReleaseListDialog extends ModalDialog {

    private static final DateFormat df = new SimpleDateFormat("yyyy/MM/dd");

    private static final String COLUMN_NAME = "Name";
    private static final String COLUMN_WEBSITE = "Target Website";
    private static final String COLUMN_SAGE_SYNC = "SAGE Sync";

    private final JLabel loadingLabel;
    private final JPanel mainPanel;
    private final DynamicTable dynamicTable;
    private final LineReleaseDialog releaseDialog;
    private List<LineRelease> releases;

    public LineReleaseListDialog() {

        setTitle("Fly Line Releases");

        releaseDialog = new LineReleaseDialog(this);

        loadingLabel = new JLabel();
        loadingLabel.setOpaque(false);
        loadingLabel.setIcon(Icons.getLoadingIcon());
        loadingLabel.setHorizontalAlignment(SwingConstants.CENTER);
        loadingLabel.setVerticalAlignment(SwingConstants.CENTER);

        mainPanel = new JPanel(new BorderLayout());
        mainPanel.add(loadingLabel, BorderLayout.CENTER);

        add(mainPanel, BorderLayout.CENTER);

        dynamicTable = new DynamicTable(true, false) {
            @Override
            public Object getValue(Object userObject, DynamicColumn column) {
                LineRelease release = (LineRelease) userObject;
                if (release != null) {
                    if (COLUMN_NAME.equals(column.getName())) {
                        return release.getName();
                    }
                    else if (COLUMN_WEBSITE.equals(column.getName())) {
                        return release.getTargetWebsite();
                    }
                    else if (COLUMN_SAGE_SYNC.equals(column.getName())) {
                        return release.isSageSync();
                    }
                    else {
                        throw new IllegalStateException("No such column: "+column.getName());
                    }
                }
                return null;
            }

            @Override
            protected JPopupMenu createPopupMenu(MouseEvent e) {
                JPopupMenu menu = super.createPopupMenu(e);

                if (menu != null) {
                    JTable table = getTable();
                    ListSelectionModel lsm = table.getSelectionModel();
                    if (lsm.getMinSelectionIndex() != lsm.getMaxSelectionIndex()) {
                        return menu;
                    }

                    final LineRelease release = (LineRelease) getRows().get(table.getSelectedRow()).getUserObject();

                    JMenuItem editItem = new JMenuItem("Edit");
                    editItem.addActionListener(e1 -> releaseDialog.showForRelease(release));
                    menu.add(editItem);

                    JMenuItem deleteItem = new JMenuItem("Delete");
                    deleteItem.addActionListener(e12 -> {

                        int result = JOptionPane.showConfirmDialog(LineReleaseListDialog.this, "Are you sure you want to delete release '"
                                + release.getName() + "'? This will not remove anything already published to the web.",
                                "Delete Release", JOptionPane.OK_CANCEL_OPTION);
                        if (result != 0) {
                            return;
                        }

                        UIUtils.setWaitingCursor(LineReleaseListDialog.this);

                        SimpleWorker worker = new SimpleWorker() {

                            @Override
                            protected void doStuff() throws Exception {
                                DomainMgr.getDomainMgr().getModel().remove(release);
                            }

                            @Override
                            protected void hadSuccess() {
                                UIUtils.setDefaultCursor(LineReleaseListDialog.this);
                                loadReleases();
                            }

                            @Override
                            protected void hadError(Throwable error) {
                                FrameworkAccess.handleException(error);
                                UIUtils.setDefaultCursor(LineReleaseListDialog.this);
                                loadReleases();
                            }
                        };
                        worker.execute();
                    });
                    menu.add(deleteItem);
                }

                return menu;
            }

            @Override
            protected void rowDoubleClicked(int row) {
                final LineRelease release = (LineRelease) getRows().get(row).getUserObject();
                releaseDialog.showForRelease(release);
            }

            @Override
            public Class<?> getColumnClass(int column) {
                DynamicColumn dc = getColumns().get(column);
                if (dc.getName().equals(COLUMN_SAGE_SYNC)) {
                    return Boolean.class;
                }
                return super.getColumnClass(column);
            }

            @Override
            protected void valueChanged(DynamicColumn dc, int row, Object data) {
                if (dc.getName().equals(COLUMN_SAGE_SYNC)) {
                    final Boolean selected = data == null ? Boolean.FALSE : (Boolean) data;
                    DynamicRow dr = getRows().get(row);
                    final LineRelease release = (LineRelease) dr.getUserObject();
                    SimpleWorker worker = new SimpleWorker() {

                        @Override
                        protected void doStuff() throws Exception {
                            release.setSageSync(selected);
                            DomainMgr.getDomainMgr().getModel().update(release);
                        }

                        @Override
                        protected void hadSuccess() {
                        }

                        @Override
                        protected void hadError(Throwable error) {
                            FrameworkAccess.handleException(error);
                        }
                    };
                    worker.execute();
                }
            }
        };

        dynamicTable.addColumn(COLUMN_NAME);
        dynamicTable.addColumn(COLUMN_WEBSITE);
        dynamicTable.addColumn(COLUMN_SAGE_SYNC).setEditable(true);

        JButton addButton = new JButton("Add new");
        addButton.setToolTipText("Add a new fly line release definition");
        addButton.addActionListener(e -> releaseDialog.showForNewRelease());

        JButton okButton = new JButton("OK");
        okButton.setToolTipText("Close this dialog");
        okButton.addActionListener(e -> setVisible(false));

        JPanel buttonPane = new JPanel();
        buttonPane.setLayout(new BoxLayout(buttonPane, BoxLayout.LINE_AXIS));
        buttonPane.setBorder(BorderFactory.createEmptyBorder(0, 10, 10, 10));
        buttonPane.add(addButton);
        buttonPane.add(Box.createHorizontalGlue());
        buttonPane.add(okButton);

        add(buttonPane, BorderLayout.SOUTH);
    }

    public void showDialog() {

        loadReleases();

        Component mainFrame = FrameworkAccess.getMainFrame();
        setPreferredSize(new Dimension((int) (mainFrame.getWidth() * 0.4), (int) (mainFrame.getHeight() * 0.4)));

        ActivityLogHelper.logUserAction("LineReleaseListDialog.showDialog");

        // Show dialog and wait
        packAndShow();
    }

    private void loadReleases() {

        mainPanel.removeAll();
        mainPanel.add(loadingLabel, BorderLayout.CENTER);

        this.releases = new ArrayList<>();

        SimpleWorker worker = new SimpleWorker() {

            @Override
            protected void doStuff() throws Exception {
                for (LineRelease releaseEntity : DomainMgr.getDomainMgr().getModel().getLineReleases()) {
                    releases.add(releaseEntity);
                }
            }

            @Override
            protected void hadSuccess() {

                // Update the attribute table
                dynamicTable.removeAllRows();
                for (LineRelease releaseEntity : releases) {
                    dynamicTable.addRow(releaseEntity);
                }

                dynamicTable.updateTableModel();
                mainPanel.removeAll();
                mainPanel.add(dynamicTable, BorderLayout.CENTER);
                mainPanel.revalidate();
            }

            @Override
            protected void hadError(Throwable error) {
                FrameworkAccess.handleException(error);
                mainPanel.removeAll();
                mainPanel.add(dynamicTable, BorderLayout.CENTER);
                mainPanel.revalidate();
            }
        };
        worker.execute();
    }

    public void refresh() {
        loadReleases();
    }

    public void totalRefresh() {
        throw new UnsupportedOperationException();
    }

    List<LineRelease> getReleases() {
        return releases;
    }

}
