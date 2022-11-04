# Configuring your Visual Scripting preferences

You can configure specific preferences in Visual Scripting to control the behavior of the Graph window and your nodes. 

To configure your preferences for Visual Scripting, go to **Edit** &gt; **Preferences**, and then in the **Preferences** window, select **Visual Scripting**. 

## Visual Scripting preferences reference 

The following tables describe the preferences you can configure in Visual Scripting.

### Core preferences

The following preferences control general behaviors across all graph types in Visual Scripting. 

<table>
<thead>
<tr>
<th><strong>Preference</strong></th>
<th><strong>Description</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td><strong>Dim Inactive Nodes</strong></td>
<td>When enabled, any nodes that aren't yet connected to the logic flow in your graph appear dimmed in the Graph Editor. This provides you with a visual cue that in its current configuration, you aren't using the dimmed node in your graph. <br/>When disabled, Visual Scripting doesn't dim any currently disconnected or unused nodes.</td>
</tr>
<tr>
<td><strong>Dim Incompatible Nodes</strong></td>
<td>When enabled, while trying to make a connection, Visual Scripting dims all nodes that don't have a compatible connection port for your current connection. When disabled, Visual Scripting doesn't dim nodes that don't have a valid connection port.</td>
</tr>
<tr>
<td><strong>Show Variables Help</strong></td>
<td>When enabled, Visual Scripting displays a brief explanation of your selected variable scope in the Blackboard window for each variable scope. When disabled, Visual Scripting hides these explanations.</td>
</tr>
<tr>
<td><strong>Create Scene Variables</strong></td>
<td>When enabled, after creating a Scene variable in the Graph window, Visual Scripting automatically creates a GameObject called Scene Variables, with a Variables component and a Scene Variables script component. When disabled, you have to create the required components on a GameObject yourself to use your Scene variables.</td>
</tr>
<tr>
<td><strong>Show Grid</strong></td>
<td>When enabled, the Visual Scripting grid is visible in the background of the Graph Editor. When disabled, the Graph Editor hides the grid.</td>
</tr>
<tr>
<td><strong>Snap to Grid</strong></td>
<td>When enabled, nodes stick or snap to points on the Visual Scripting grid in the Graph Editor. When disabled, nodes move freely and don't automatically snap to grid points.</td>
</tr>
<tr>
<td><strong>Pan Speed</strong></td>
<td>Set a <strong>Pan Speed</strong> to control how quickly the Graph Editor moves when panning vertically using the scroll wheel.</td>
</tr>
<tr>
<td><strong>Drag Pan Speed</strong></td>
<td>Set a <strong>Drag Pan Speed</strong> to control how quickly the Graph Editor moves when moving a node to the edge of the window.</td>
</tr>
<tr>
<td><strong>Zoom Speed</strong></td>
<td>Set a <strong>Zoom Speed</strong> to control how quickly the Graph Editor zooms in or zooms out while setting a zoom level in the Graph window. For more information on zooming in the Graph Editor, see Zoom in/Zoom Out in <a href="vs-control-schemes.md#zoom-inzoom-out">Choosing a control scheme</a>.</td>
</tr>
<tr>
<td><strong>Overview Smoothing</strong></td>
<td>Set <strong>Overview Smoothing</strong> to control how gradually the Graph Editor zooms or pans after selecting the <strong>Overview</strong> option. For more information on the <strong>Overview</strong> option, see <a href="vs-interface-overview.md">The Visual Scripting interface</a>.</td>
</tr>
<tr>
<td><strong>Carry Children</strong></td>
<td>When enabled, Visual Scripting moves all connected child nodes when you move a parent node. When disabled, any connected child nodes don't move when their parent node moves. <br/> <div class="NOTE"><h5>NOTE</h5><p>You can also change this setting from the toolbar in the Graph window. For more information, see <a href="vs-interface-overview.md">The Visual Scripting interface</a>.</p></div></td>
</tr>
<tr>
<td><strong>Disable Playmode Tint</strong></td>
<td>When enabled, Visual Scripting displays all nodes in a Graph window as normal while the Editor is in Play mode. When disabled, Visual Scripting tints all nodes in a Graph window.</td>
</tr>
<tr>
<td><strong>Control Scheme</strong></td>
<td>Select your desired Visual Scripting control scheme. For more information, see <a href="vs-control-schemes.md">Choosing a control scheme</a>.</td>
</tr>
<tr>
<td><strong>Clear Graph Selection</strong></td>
<td>When enabled, Visual Scripting clears the Graph window if you select a GameObject with no set graph or graphs. When disabled, Visual Scripting keeps the last displayed graph after selecting a GameObject with no set graph or graphs. <br/> <div class="NOTE"><h5>NOTE</h5><p>Visual Scripting updates the Graph window to display the set graph on your currently selected GameObject, regardless of your chosen <strong>Clear Graph Selection</strong> setting.</p></td>
</tr>
<tr>
<td><strong>Human Naming</strong></td>
<td>When enabled, Visual Scripting converts function names from camel case. For example, <code>camelCase</code> becomes <code>Camel Case</code>. When disabled, Visual Scripting doesn't convert any names.</td>
</tr>
<tr>
<td><strong>Max Search Results</strong></td>
<td>Set a <strong>Max Search Results</strong> value to specify the maximum number of search results returned by the fuzzy finder after using the search bar.</td>
</tr>
<tr>
<td><strong>Group Inherited Members</strong></td>
<td>When enabled, in the fuzzy finder search results, Visual Scripting groups together inherited nodes from a parent or base class to your current search term. <br/> For example, an <code>Audio Source</code> is a <code>Component</code>: it has its own specific methods and nodes, but you can interact with it as a <code>Component</code> by using <code>Component</code> nodes. While searching in the fuzzy finder, Visual Scripting groups the nodes inherited from <code>Component</code> and displays them in grey. <br/>When disabled, the fuzzy finder displays nodes in the search results without grouping these inherited nodes.</td>
</tr>
<tr>
<td><strong>Developer Mode</strong></td>
<td>When enabled, Visual Scripting provides additional preferences in the Preferences window. It also adds additional features in the Graph window and other areas of the Unity Editor. For more information on the additional Developer Mode preferences, see [<a href="#additional-developer-mode-preferences">Additional Developer Mode preferences</a>.</td>
</tr>
<tr>
<td><strong>AOT Safe Mode</strong></td>
<td>When enabled, Visual Scripting excludes nodes with types that may cause problems for platforms requiring Ahead Of Time (AOT) compilation from search results in the fuzzy finder. For example, Visual Scripting excludes nodes that use the <code>Generic</code> type. <br/>When disabled, potentially problematic types aren't filtered.</td>
</tr>
</tbody>
</table>


### Script Graphs preferences 

The following preferences change the behavior of Script Graphs in the Graph window.

<table>
<thead>
<tr>
<th><strong>Preference</strong></th>
<th><strong>Description</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td><strong>Update Units Automatically</strong></td>
<td><div class="NOTE"><h5>NOTE</h5><p>This feature is experimental.</p></div>When enabled, Visual Scripting updates your node library automatically whenever it detects that a script in your project's <strong>Assets</strong> folder has changed. When disabled, you must manually update your node library. For more information on updating your node library, see <a href="vs-configuration.md">Configuring your project settings</a>.</td>
</tr>
<tr>
<td><strong>Predict Potential Null References</strong></td>
<td>When enabled, Visual Scripting's predictive debugging warns you about potential <code>null</code> value inputs in your graphs. When disabled, predictive debugging doesn't display warnings about potential <code>null</code> values. <br/> <div class="NOTE"><h5>NOTE</h5><p>Sometimes, predictive debugging may return false positives when you enable this setting.</p></div></td>
</tr>
<tr>
<td><strong>Predict Potential Missing Components</strong></td>
<td>When enabled, Visual Scripting's predictive debugging warns you about potential missing components in your graphs. When disabled, predictive debugging doesn't display warnings about potential missing components. <br/> <div class="NOTE"><h5>NOTE</h5><p>Sometimes, predictive debugging may return false positives when you enable this setting.</p></div></td>
</tr>
<tr>
<td><strong>Show Connection Values</strong></td>
<td>When enabled, the Graph Editor shows the input and output values sent between nodes while the Unity Editor is in Play mode. This can make it easier to debug your scripts. <br/>When disabled, the Graph Editor doesn't display input and output values.</td>
</tr>
<tr>
<td><strong>Predict Connection Values</strong></td>
<td>When enabled, the Graph Editor attempts to predict what input and output values will be sent between nodes while the Unity Editor is in Play mode. For example, Visual Scripting could display the value currently set for a variable in your script, though that value may change before it's used by a node. <br/>When disabled, Visual Scripting won't attempt to predict input and output values.</td>
</tr>
<tr>
<td><strong>Hide Port Labels</strong></td>
<td>When enabled, Visual Scripting doesn't display the name labels for node input and output ports. When disabled, nodes display these name labels.</td>
</tr>
<tr>
<td><strong>Animate Control Connections</strong></td>
<td>When enabled, Visual Scripting displays a droplet animation across node logic flow connections while the Editor is in Play mode. When disabled, logic flow connections aren't animated.</td>
</tr>
<tr>
<td><strong>Animate Value Connections</strong></td>
<td>When enabled, Visual Scripting displays a droplet animation across node value flow connections while the editor is in Play mode. When disabled, value flow connections aren't animated.</td>
</tr>
<tr>
<td><strong>Skip Context Menu</strong></td>
<td>When enabled, a right-click inside the Graph window opens the Visual Scripting fuzzy finder. To access the context menu, you must Shift + right-click. <br/>When disabled, right-click without a node selected to open the fuzzy finder; right-click with a node selected to open the context menu.</td>
</tr>
</tbody>
</table>


### State Graphs preferences

The following preferences change the behavior of State Graphs in the Graph window. 
<table>
<thead>
<tr>
<th><strong>Preference</strong></th>
<th><strong>Description</strong></th>
</tr>
</thead>
<tbody>
<tr>
<td><strong>States Reveal</strong></td>
<td>Use the dropdown to choose when a Script State node displays a list of events from its graph: <br/>
<ul>
<li><strong>Never</strong>: Script State nodes never display their list of events.</li>
<li><strong>Always</strong>: Script State nodes always display their list of events.</li>
<li><strong>On Hover</strong>: Script State nodes only display their list of events when you hover over the node in the Graph window.</li>
<li><strong>On Hover with Alt</strong>: Script State nodes only display their list of events when you hover over the node while holding Alt.</li>
<li><strong>When Selected</strong>: Script State nodes only display their list of events when you select them in the Graph window.</li>
<li><strong>On Hover or Selected</strong>: Script State nodes display their list of events when you hover over the node, or when you select it in the Graph window.</li>
<li><strong>On Hover with Alt or Selected</strong>: Script State nodes display their list of events when you hover over the node while holding Alt, or when you select it in the Graph window.</li>
</ul>
You might want to change this setting if you have a lot of Script State nodes in your State Graphs.</td>
</tr>
<tr>
<td><strong>Transitions Reveal</strong></td>
<td>Use the dropdown to choose when a transition displays a list of events from its graph: <br/>
<ul>
<li><strong>Never</strong>: Transitions never display a list of events.</li>
<li><strong>Always</strong>: Transitions always display a list of events.</li>
<li><strong>On Hover</strong>: Transitions only display a list of events when you hover over the transition in the Graph window.</li>
<li><strong>On Hover with Alt</strong>: Transitions only display a list of events when you hover over the transition while holding Alt.</li>
<li><strong>When Selected</strong>: Transitions only display a list of events when you select them in the Graph window.</li>
<li><strong>On Hover or Selected</strong>: Transitions display a list of events when you hover over the transition, or when you select it in the Graph window.</li>
<li><strong>On Hover with Alt or Selected</strong>: Transitions display a list of events when you hover over the transition while holding Alt, or when you select it in the Graph window.</li>
</ul>
You might want to change this setting if you have a lot of transitions in your State Graphs.</td>
</tr>
<tr>
<td><strong>Transitions End Arrow</strong></td>
<td>When enabled, Visual Scripting adds an arrow to the end of each transition connection in your State Graphs. When disabled, transition connections display as simple lines. <br/>If you have a lot of transitions in your State Graphs, you might want to leave this setting disabled.</td>
</tr>
<tr>
<td><strong>Animate Transitions</strong></td>
<td>When enabled, Visual Scripting displays a droplet animation across transition connections when the Editor is in Play mode. When disabled, transition connections aren't animated.</td>
</tr>
</tbody>
</table>

### Additional Developer Mode preferences

> [!NOTE]
> You can only access the following preferences after you have enabled **Developer Mode**. 

These Developer Mode preferences provide help with developing extensions or custom nodes for Visual Scripting. Their continued support in the Visual Scripting package isn't guaranteed. 

|**Preference** |**Description** |
|:---|:---|
|__Debug__ | When enabled, Visual Scripting adds additional logging and visual overlays to help you debug element rendering in the Graph window. For example, if you created a custom node, enabling this setting might help you to create your desired UI.|
|__Track Metadata State__ | When enabled, Visual Scripting adds more information to its logging to help assist with debugging.|
|__Debug Inspector UI__ | When enabled, Visual Scripting adds more detail and additional overlays. The information available is greater than what Visual Scripting provides with the **Debug** setting, and affects more areas of the Editor's UI. You should only enable this setting if you need more in-depth debugging feedback.|
