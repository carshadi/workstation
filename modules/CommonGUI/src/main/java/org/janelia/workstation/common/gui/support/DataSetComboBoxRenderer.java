package org.janelia.workstation.common.gui.support;

import java.awt.Component;

import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.ListCellRenderer;
import javax.swing.SwingConstants;

import org.janelia.model.domain.sample.DataSet;

/**
 * A combo-box renderer for Data Set selection. 
 * 
 * @author <a href="mailto:rokickik@janelia.hhmi.org">Konrad Rokicki</a>
 */
public class DataSetComboBoxRenderer extends JLabel implements ListCellRenderer<DataSet> {

    public DataSetComboBoxRenderer() {
        setOpaque(true);
        setHorizontalAlignment(SwingConstants.LEFT);
        setVerticalAlignment(SwingConstants.CENTER);
    }

    @Override
    public Component getListCellRendererComponent(JList<? extends DataSet> list, DataSet dataSetEntity, int index, boolean isSelected, boolean cellHasFocus) {

        if (dataSetEntity == null) {
            setIcon(null);
            setText("");
            return this;
        }

        if (isSelected) {
            setBackground(list.getSelectionBackground());
            setForeground(list.getSelectionForeground());
        }
        else {
            setBackground(list.getBackground());
            setForeground(list.getForeground());
        }

        setIcon(Icons.getIcon("folder_database.png"));
        setText(dataSetEntity.getIdentifier());

        return this;
    }
}