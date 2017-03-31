package org.janelia.it.workstation.gui.framework.outline;

import java.awt.Color;
import java.awt.Component;
import java.awt.FlowLayout;
import java.text.DateFormat;
import java.text.SimpleDateFormat;

import javax.swing.BorderFactory;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTree;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;
import javax.swing.tree.TreeCellRenderer;

import org.janelia.it.jacs.model.entity.Entity;
import org.janelia.it.jacs.model.entity.EntityConstants;
import org.janelia.it.jacs.model.entity.EntityData;
import org.janelia.it.jacs.shared.utils.EntityUtils;
import org.janelia.it.workstation.api.entity_model.management.ModelMgrUtils;
import org.janelia.it.workstation.gui.util.Icons;

/**
 * Special tree cell renderer for generic Entity trees.
 *
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class EntityTreeCellRenderer extends DefaultTreeCellRenderer implements TreeCellRenderer {

    protected static final Color typeLabelColor = new Color(149, 125, 71);
    protected static final Color metaLabelColor = new Color(128, 128, 128);
    protected static final Color highlightColor = new Color(205, 157, 250);
    protected static final DateFormat df = new SimpleDateFormat("yyyy/MM/dd HH:mm");

    protected JLabel titleLabel;
    protected JLabel typeLabel;
    protected JLabel metaLabel;
    protected JPanel cellPanel;
    protected Color foregroundSelectionColor;
    protected Color foregroundNonSelectionColor;
    protected Color backgroundSelectionColor;
    protected Color backgroundNonSelectionColor;
    protected DefaultTreeCellRenderer defaultRenderer = new DefaultTreeCellRenderer();

    public EntityTreeCellRenderer() {

        cellPanel = new JPanel(new FlowLayout(FlowLayout.LEFT));
        cellPanel.setOpaque(false);

        titleLabel = new JLabel(" ");
        titleLabel.setOpaque(true);
        titleLabel.setBorder(BorderFactory.createEmptyBorder(2, 2, 2, 0));
        cellPanel.add(titleLabel);

        typeLabel = new JLabel(" ");
        typeLabel.setForeground(EntityTreeCellRenderer.typeLabelColor);
        cellPanel.add(typeLabel);

        metaLabel = new JLabel(" ");
        metaLabel.setForeground(EntityTreeCellRenderer.metaLabelColor);
        cellPanel.add(metaLabel);

        foregroundSelectionColor = defaultRenderer.getTextSelectionColor();
        foregroundNonSelectionColor = defaultRenderer.getTextNonSelectionColor();
        backgroundSelectionColor = defaultRenderer.getBackgroundSelectionColor();
        backgroundNonSelectionColor = defaultRenderer.getBackgroundNonSelectionColor();
    }

    @Override
    public Component getTreeCellRendererComponent(JTree tree, Object value, boolean selected, boolean expanded, boolean leaf, int row, boolean hasFocus) {
        Component returnValue = null;
        if ((value != null) || (value instanceof DefaultMutableTreeNode)) {

            // Set the colors
            if (selected) {
                titleLabel.setForeground(foregroundSelectionColor);
                titleLabel.setBackground(backgroundSelectionColor);
            }
            else {
                titleLabel.setForeground(foregroundNonSelectionColor);
                titleLabel.setBackground(backgroundNonSelectionColor);
            }

            // Support drag and drop 
            JTree.DropLocation dropLocation = tree.getDropLocation();
            if (dropLocation != null
                    && dropLocation.getChildIndex() == -1
                    && tree.getRowForPath(dropLocation.getPath()) == row) {
                titleLabel.setForeground(foregroundSelectionColor);
                titleLabel.setBackground(backgroundSelectionColor);
            }

            // Set the default icon
            cellPanel.setEnabled(tree.isEnabled());
            if (leaf) {
                titleLabel.setIcon(getLeafIcon());
            }
            else if (expanded) {
                titleLabel.setIcon(getOpenIcon());
            }
            else {
                titleLabel.setIcon(getClosedIcon());
            }

            // Set everything else based on the entity properties
            DefaultMutableTreeNode node = (DefaultMutableTreeNode) value;
            Object userObject = node.getUserObject();

            titleLabel.setText("");
            typeLabel.setText("");
            titleLabel.setIcon(null);
            metaLabel.setText("");

            EntityData ed = null;
            Entity entity = null;

            if (userObject instanceof EntityData) {
                ed = (EntityData) userObject;
                entity = (Entity) ed.getChildEntity();
            }
            else if (userObject instanceof Entity) {
                entity = (Entity) userObject;
            }

            if (entity != null) {

                String entityAttrName = ed.getEntityAttrName() == null ? "" : ed.getEntityAttrName();
                String entityTypeName = entity.getEntityTypeName() == null ? "" : entity.getEntityTypeName();

                // Set the labels
                titleLabel.setText(entity.getName());
                titleLabel.setIcon(Icons.getIcon(entity));
                titleLabel.setToolTipText(entityTypeName);

                String dateStr = entity.getUpdatedDate() == null ? "" : df.format(entity.getUpdatedDate());
                String ownerStr = EntityUtils.getNameFromSubjectKey(entity.getOwnerKey());

                if (ed != null && (entityAttrName.equals(EntityConstants.ATTRIBUTE_RESULT) || entityTypeName.equals(EntityConstants.TYPE_PIPELINE_RUN))) {
                    typeLabel.setText(dateStr + " " + ownerStr);
                }
                else {
                    typeLabel.setText(ownerStr);
                }

                if (entityTypeName.equals(EntityConstants.TYPE_NEURON_FRAGMENT_COLLECTION)) {
                    metaLabel.setText("(" + ModelMgrUtils.getAccessibleChildren(entity).size() + " fragments)");
                }

                if (entityTypeName.equals(EntityConstants.TYPE_CURATED_NEURON_COLLECTION)) {
                    metaLabel.setText("(" + ModelMgrUtils.getAccessibleChildren(entity).size() + " items)");
                }

                if (isHighlighted(entity)) {
                    titleLabel.setBackground(highlightColor);
                }

            }

            returnValue = cellPanel;
        }
        if (returnValue == null) {
            returnValue = defaultRenderer.getTreeCellRendererComponent(tree, value, selected, expanded, leaf, row, hasFocus);
        }
        return returnValue;
    }

    protected boolean isHighlighted(Entity entity) {
        return false;
    }
}