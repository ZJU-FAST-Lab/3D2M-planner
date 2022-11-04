# The Visual Scripting interface

Visual Scripting's main window is the Graph window. 

![The Visual Scripting Graph window, showing a Script Graph.](images\vs-graph-window-overview.png)

The Graph window has five main elements: 

- The [**Graph Editor**](#the-graph-editor), where you create, arrange, and connect nodes. 
- The [**fuzzy finder**](#the-fuzzy-finder), which you can use to find nodes and add them to your graph.
- The [**Graph toolbar**](#the-graph-toolbar), where you can change settings specific to your view in the Graph Editor and perform some common layout operations. 
- The [**Graph Inspector**](#the-graph-inspector), where you can view detailed information about your nodes and configure additional settings for your graph. 
- The [**Blackboard**](#the-blackboard), where you can define and edit variables to use in your graphs.  

## The Graph Editor

The Graph Editor is the center editing area of the Graph window.

![A view of the Graph Editor with multiple nodes and connections in a Script Graph](images\vs-graph-editor.png)

You can use the Graph Editor to create your Visual Scripting graphs by manipulating nodes and connections. 

You can change some default shortcuts and behaviors for interacting with the Graph Editor by changing your control scheme. For more information on the available control schemes in Visual Scripting, see [Choosing a control scheme](vs-control-schemes.md).

## The fuzzy finder

The fuzzy finder is a searchable menu that lists every node available to you in Visual Scripting. You can open the fuzzy finder by right-clicking anywhere in the Graph Editor: 

![A view of the Graph Editor, with the fuzzy finder menu open](images\vs-fuzzy-finder.png)

You can search for a node by name using the Search bar, or open a category from the list to find related nodes. For example, you can find any nodes related to creating or manipulating variables in the **Variables** category.

You can also add new nodes to Visual Scripting from your own code, from other packages, or from other Unity features. For more information on adding new nodes to the fuzzy finder, see [Configuring your project settings](vs-configuration.md).

## The Graph toolbar 

The Graph toolbar lets you display or hide the Graph Inspector and Blackboard. When navigating through nested Script Graphs or State Graphs, the Graph toolbar also includes a breadcrumb trail showing your current location. You can select a graph from the trail to return to that graph file.

You can also configure some additional settings that control how nodes display in the Graph Editor.

![The Visual Scripting Graph toolbar](images\vs-toolbar.png)


|**Property** | **Description** |
| :--- | :--- |
|__Zoom__ | Set your preferred zoom level for viewing your graphs in the Graph Editor. |
|__Relations__ | When enabled, nodes in Script Graphs display their inner flow connections. For example, in a standard `Multiply` node, the data flow between the two input ports would merge together into the single output port on the node. <br/>When disabled, Visual Scripting hides these connections. |
|__Values__ | When enabled, the Graph Editor shows the input and output values sent between nodes while the Unity Editor is in Play mode. This can make it easier to debug your scripts. <br/>When disabled, the Graph Editor doesn't display input and output values. |
|__Dim__ | When enabled, any nodes that aren't yet connected to the control flow in your graph appear dimmed in the Graph Editor. This provides you with a visual cue that in its current configuration, you aren't using the dimmed node in your graph. <br/>When disabled, Visual Scripting doesn't dim unused nodes. |
|__Carry__ | When enabled, while moving a parent node, Visual Scripting also moves all connected child nodes. <br/>When disabled, any connected child nodes don't move when their parent node moves. |
|__Align__ | With multiple nodes in your graph selected, you can choose to align those nodes based on one of the available options. For a detailed description of each option, see [the Fields table on the AlignOperation page in the Scripting API reference](xref:Unity.VisualScripting.AlignOperation).|
|__Distribute__ | With multiple nodes in your graph selected, you can choose an option to evenly distribute the available space between your selected nodes. For a detailed description of each option, see [the Fields table in the DistributeOperation page in the Scripting API reference](xref:Unity.VisualScripting.DistributeOperation). |
|__Overview__ | When selected, Visual Scripting pans and zooms to fit all the elements of your current graph within the Graph Editor. |
|__Full Screen__ | When selected, if you've docked your Graph window in the Unity Editor, Visual Scripting maximizes your Graph window to take up the entire Unity Editor window. |


You can change some Graph toolbar settings from your preferences for Visual Scripting, or change how these settings behave. For example, you can control how fast the Graph Editor zooms in and out when setting your zoom level. For more information, see [Configuring your preferences](vs-set-preferences.md).

## The Graph Inspector 

The Graph Inspector provides additional information about an open graph, or about any node you select in the Graph Editor.

![The Graph Inspector with a Switch on String node selected](images\vs-graph-inspector-switch-node.png)

If a node requires additional configuration, you can use the Graph Inspector to set these values. 

To display or hide the Graph Inspector, select **Graph Inspector** (![The Graph Inspector icon](images\vs-graph-inspector-icon.png)) from the toolbar.

To move the Graph Inspector to the other side of the Graph window, select either **Dock Right** (![The Dock Right button](images\VS-RightSide.png)) or **Dock Left** (![The Dock Left button](images\VS-LeftSide.png)).

## The Blackboard 

The Blackboard provides options for configuring and managing variables in your graph. The Blackboard divides variables into five distinct scopes, across five tabs: **Graph**, **Object**, **Scene**, **App**, and **Saved**. 

![The Blackboard open to the Graph variables tab](images\vs-blackboard.png)

For more information on the available variable scopes in Visual Scripting, see [Variables](vs-variables.md).

To display or hide the Blackboard, select **Blackboard** (![The Blackboard icon](images\vs-blackboard-icon.png)) from the toolbar.

To move the Blackboard to the other side of the Graph window, select either **Dock Right** (![The Dock Right button](images\VS-RightSide.png)) or **Dock Left** (![The Dock Left button](images\VS-LeftSide.png)).

