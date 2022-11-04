# Choosing a Visual Scripting control scheme

You can choose from two different control schemes in Visual Scripting. Each control scheme changes how you can interact with your graphs in the Graph Editor: 

|**Action** |**Default Control Scheme** |**Alternate Control Scheme** |
|:---|:---|:---|
|__Pan__| Middle-click and drag | Middle-click and drag |
|__Pan Vertically__ | Scroll | N/A |
|__Zoom In/Zoom Out__ | Ctrl/Cmd + Scroll | Scroll |
|__Frame Selected__ | Home | Home |
|__Frame All__ | Home | Home |
|__Create Selection__ | Click and drag | Click and drag |
|__Select All__ | Ctrl/Cmd + A | Ctrl/Cmd + A |
|__Open Context Menu__| Right-click <br/>Ctrl + click (MacOS) <br/>Ctrl/Cmd + E | Right-click <br/>Ctrl + click (MacOS) <br/>Ctrl/Cmd + E |
|__Create Node Group__| Ctrl/Cmd + click and drag | Ctrl/Cmd + click and drag |
|__Copy Selected__| Ctrl/Cmd + C | Ctrl/Cmd + C |
|__Paste Selection__| Ctrl/Cmd + V | Ctrl/Cmd + V |
|__Cut Selected__| Ctrl/Cmd + X | Ctrl/Cmd + X |
|__Duplicate Selected__| Ctrl/Cmd + D | Ctrl/Cmd + D |
|__Delete Selected__| Del | Del |
|__Maximize Graph Window__| Shift + Space <br/>Double-click | Shift + Space <br/>Double-click |
|__Move Group Without Child Nodes__| Alt + Click and drag the group's Title bar | Ctrl/Cmd + Click and drag the group's Title bar |
|__Move Node on One Axis__| Shift + Click and drag vertically or horizontally | Shift + Click and drag vertically or horizontally |


## Pan 

By panning, you can move the viewable area in the Graph Editor to any part of your graph. 

## Pan Vertically 

When using the Default control scheme, you can pan your view in the Graph Editor vertically by scrolling. 

## Zoom In/Zoom Out 

You can change the zoom level in the Graph window to control how much of your graph is visible in the Graph Editor. 

You can also set your zoom level using the toolbar in the Graph window. For more information, see [The Visual Scripting interface](vs-interface-overview.md).

## Frame Selected 

After selecting a node or another item in your graph, press Home to center your selected item in the Graph Editor:

![A Graph window, with a Set Variable node selected and framed in the Graph Editor](images\vs-frame-selected.png)

## Frame All 

With no nodes or items selected, press Home to center your entire graph in the Graph Editor. Your zoom level automatically adjusts to accommodate the size of your graph: 

![A Graph window, with its Script Graph framed in the Graph Editor](images\vs-frame-all.png)

## Create Selection

You can create a selection of multiple nodes or items in your graph by clicking in an empty space and dragging to create a box around the nodes or items you want to select: 

![A Graph window, with a selection created around 4 nodes in the Graph Editor](images\vs-create-selection.png)

With multiple items selected, you can click and drag a single item to move your entire selection.

## Select All 

You can quickly select all items in your current graph. 

## Open Context Menu

You can open the context menu to perform certain actions on State Graphs, like creating new states, adding transitions, or manipulating a selection in a Script Graph: 

![A Graph window, with a Script Graph, displaying the context menu in the Graph Editor](images\vs-context-menu.png)

## Create Node Group 

You can create a group of nodes to keep related sections of your graph together, or move multiple nodes at a time. You can name your node groups to help keep them organized: 

![A Graph window, with a node group called Variable Input created in the Graph Editor](images\vs-node-group.png)

## Copy Selected 

You can copy your current selection to move it to another graph, or another location on your current graph. 

## Paste Selection

You can paste the contents of a copied selection into your graph. You can also paste a selection you cut from a graph. 

## Cut Selected

You can cut your current selection to move it to another graph, or another location on your current graph. 

## Duplicate Selected 

You can quickly create a copy of your current selection to use elsewhere in your current graph: 

![A Graph window with a set of nodes that have been duplicated in the Graph Editor](images\vs-duplicate-selection.png)

## Delete Selected

You can delete your current selection to remove it from your graph. 

## Maximize Graph Window 

When you've docked your Graph window in the Unity Editor, you can maximize your Graph window to take up the entire Unity Editor window. 

## Move Group Without Child Nodes 

You can move a group in your graph without moving the nodes contained inside that group: 

![A Graph window with a node group moved to another location without its child nodes](images\vs-move-group.png)

## Move Node on One Axis 

You can choose to move a node in only one direction at a time in the Graph Editor, either vertically or horizontally. 