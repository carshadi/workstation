package org.janelia.workstation.gui.large_volume_viewer.action;

import java.net.MalformedURLException;
import java.net.URL;
import java.util.LinkedList;
import java.util.Vector;
import java.util.prefs.Preferences;

import javax.swing.JMenu;
import org.janelia.workstation.gui.large_volume_viewer.listener.UrlLoadListener;

/**
 *  Memorized list of recently opened files for display in a menu.
 *  
 * @author brunsc
 *
 */
public class RecentFileList 
{
	static final int maxItems = 10;
	// private static final Logger log = LoggerFactory.getLogger(RecentFileList.class);
	
	private Preferences prefs;
	private JMenu menu;
    private UrlLoadListener urlLoadListener;
	private LinkedList<RecentFileAction> actions = new LinkedList<>();

	public RecentFileList(JMenu menu)
	{
		this.menu = menu;
		prefs = Preferences.userNodeForPackage(this.getClass());
		loadRecentFiles();
        updateMenu();
	}

    /**
     * @param urlLoadListener the urlLoadListener to set
     */
    public void setUrlLoadListener(UrlLoadListener urlLoadListener) {
        this.urlLoadListener = urlLoadListener;
        for (RecentFileAction action: actions) {
        	// Propagate signal
            action.setUrlLoadListener(urlLoadListener);
        }
    }
	
	public void add(URL url) {
		// log.info("Adding recent file" + url);
		RecentFileAction action = new RecentFileAction(url);
		if ((actions.size() > 0) && (action.equals(actions.getFirst())))
			return; // already there at the top of the list
		if (actions.contains(action))
			actions.remove(action); // it appears later in the list. remove it.
		// Latest url is first in the list
		actions.addFirst(action);
		// Restrict maximum list size
		while (actions.size() > maxItems)
			actions.removeLast();
		// Expose results of add
		saveRecentFiles();
		updateMenu();
	}
	
	private void loadRecentFiles() {
		int index = 1;
		Vector<RecentFileAction> a = new Vector<RecentFileAction>();
		while(true) {
			String key = "RECENT_FILE" + index;
			String value = prefs.get(key, "");
			if (value.isEmpty())
				break; // empty string signals end of list
			try {
				URL url = new URL(value);
				RecentFileAction action = new RecentFileAction(url);
                action.setUrlLoadListener(urlLoadListener);
				a.add(action);
			} catch (MalformedURLException e) {
				// ignore bad URLs
			}
			++index;
		}
		actions.clear();
		actions.addAll(a);
	}

	private void saveRecentFiles() {
		int index = 1;
		for (RecentFileAction action : actions) {
			String key = "RECENT_FILE" + index;
			String value = action.getUrl().toString();
			if (value == "")
				continue;
			prefs.put(key, value);
			++index;
		}
		// Add one blank entry as a terminator
		prefs.put("RECENT_FILE"+index, "");
	}

	private void updateMenu() {
		// Tear the menu down, then build it back up
		menu.removeAll();
		if (actions.size() == 0) {
			// This menu is empty
			menu.setEnabled(false);
			menu.setVisible(false);
			return;
		}
		// This menu is not empty
		menu.setVisible(true);
		menu.setEnabled(true);
		for (RecentFileAction a : actions)
			menu.add(a);		
	}

}
