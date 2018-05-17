package org.janelia.it.workstation.gui.task_workflow;

/**
 *
 * @author schauderd
 */


import java.awt.Color;
import java.awt.Image;
import java.awt.datatransfer.DataFlavor;
import java.awt.datatransfer.Transferable;
import java.awt.datatransfer.UnsupportedFlavorException;
import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Observable;
import java.util.Observer;
import org.janelia.console.viewerapi.ObservableInterface;
import org.janelia.console.viewerapi.model.NeuronSet;
import org.janelia.console.viewerapi.model.VantageInterface;
import org.janelia.console.viewerapi.model.HortaMetaWorkspace;
import org.janelia.console.viewerapi.model.NeuronModel;
import org.openide.ErrorManager;
import org.openide.nodes.AbstractNode;
import org.openide.nodes.ChildFactory;
import org.openide.nodes.Children;
import org.openide.nodes.Node;
import org.openide.nodes.PropertySupport;
import org.openide.nodes.Sheet;
import org.openide.util.Exceptions;
import org.openide.util.ImageUtilities;
import org.openide.util.datatransfer.PasteType;
import org.openide.util.lookup.Lookups;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 */
public class ReviewGroupNode extends AbstractNode
{
    private final Logger logger = LoggerFactory.getLogger(this.getClass());
    
    public ReviewGroupNode(ReviewGroup reviewGroup) {
        super(Children.create(new ReviewGroupNodeFactory(reviewGroup), true), Lookups.singleton(reviewGroup));
        updateDisplayName();
       
    }
    
    private void updateDisplayName() {
        setDisplayName("Scene"); //  (" + workspace.getNeuronSets().size() + " neurons)");
    }
    
     private static class ReviewGroupNodeFactory extends ChildFactory<ReviewPoint>
    {
        private List<ReviewPoint> pointList;
        
        public ReviewGroupNodeFactory(ReviewGroup group) {
            this.pointList = group.getPointList();
        }

        @Override
        protected boolean createKeys(List<ReviewPoint> toPopulate)
        {
            toPopulate.addAll(pointList);
            return true;
        }

        @Override
        protected Node createNodeForKey(ReviewPoint key) {
            return new ReviewPointNode(key);
        }
    }
    
    @Override
    public Image getIcon(int type) {
        return ImageUtilities.loadImage("org/janelia/horta/images/brain-icon2.png");
    }
    
    @Override
    public Image getOpenedIcon(int i) {
        return getIcon(i);
    }
    
    public Integer getSize() {return 1;}
    
    @Override 
    protected Sheet createSheet() { 
        Sheet sheet = Sheet.createDefault(); 
        Sheet.Set set = Sheet.createPropertiesSet(); 
        try { 
            Property prop;
            // size
            prop = new PropertySupport.Reflection(this, String.class, "notes", null); 
            prop.setName("notes"); 
            set.put(prop); 
        } 
        catch (NoSuchMethodException ex) {
            ErrorManager.getDefault(); 
        } 
        sheet.put(set); 
        return sheet; 
    }

}

