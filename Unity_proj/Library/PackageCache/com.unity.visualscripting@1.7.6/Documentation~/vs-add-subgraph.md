# Add a Subgraph to a Script Graph

A Subgraph is a Script Graph nested inside of another Script Graph. A Subgraph appears as a single node inside the parent Script Graph. 

You can add a Subgraph to a Script Graph in two ways: create an entirely new Script Graph, or add an existing Script Graph file.

## Add a new Subgraph to a Script Graph

To add a new blank Subgraph to an existing Script Graph: 

1. With a Script Graph [open in the Graph window](vs-open-graph-edit.md), in the Graph Editor, right-click an empty area to open the fuzzy finder and go to **Nesting** &gt; **Subgraph**. 

2. In the Graph Inspector, choose the **Source** for your Subgraph: 

   - **Embed**: The Subgraph only exists on the Subgraph node. You can only modify the Subgraph from the node in its parent graph.
   - **Graph**: The Subgraph exists in a separate file. You can modify the Subgraph outside of its parent graph and reuse the graph in other areas of your application. 

   > [!TIP]
   > If the Graph Inspector isn't visible in the Graph window, select **Graph Inspector** (![The Graph Inspector icon](images/vs-graph-inspector-icon.png)) from the toolbar.

3. If you chose **Graph**, select **New**, enter a name for your graph file, and choose where you want to save it. Select **Save**. 

![An image of the Graph window, showing a new blank Subgraph node added to a Script Graph](images/vs-blank-graph-subgraph-example.png)

To open your new Subgraph for editing, select **Edit Graph**. 

## Add an existing Script Graph as a Subgraph

To add an existing graph file as a Subgraph in a Script Graph: 

> [!NOTE]
> You can't nest a Script Graph as a Subgraph in its own graph file. 

1. With a Script Graph [open in the Graph window](vs-open-graph-edit.md), in the Graph Editor, right-click an empty area to open the fuzzy finder and go to **Nesting** &gt; **Subgraph**. 

2. In the Graph Inspector, set your **Source** to **Graph**. 

   > [!TIP]
   > If the Graph Inspector isn't visible in the Graph window, select **Graph Inspector** (![The Graph Inspector icon](images/vs-graph-inspector-icon.png)) from the toolbar.

3. In the **Graph** field, select the target icon and choose a compatible Script Graph from your project. You can also click and drag a Script Graph file from your Project window and drop it into the **Graph** field. 

![An image of the Graph window, showing a new Subgraph node created from an existing Script Graph added to another Script Graph file](images/vs-existing-graph-example-subgraph.png)

> [!TIP] 
> For a faster way to add a Script Graph as a Subgraph, click and drag the Script Graph from your Project window into the Graph Editor to automatically create a Subgraph node. 

## Next steps

Once you've added a Subgraph to your Script Graph, define its Input and Output Triggers and Input and Output Data. For more information, see [Add a Trigger or Data port to a Script Graph](vs-add-triggers-data-graph.md).

