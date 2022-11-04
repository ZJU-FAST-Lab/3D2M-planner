# Create and assign a graph to an existing GameObject

You can use the empty graph creation flow to create a new graph file and assign it to an existing GameObject in your project. 

![The Empty Graph Creation Flow window](images\vs-empty-graph-create-flow.png)

For more information on other ways to create a graph file, see [Creating a new graph file](vs-create-graph.md).

To create a new graph and assign it to an existing GameObject:

1. In the Editor's Hierarchy window, select the GameObject where you want to assign your new graph. If the Hierarchy window isn't visible, go to **Window** &gt; **General** &gt; **Hierarchy**, or press CTRL + 4 (macOS: Cmd + 4). 

2. Go to **Window** &gt; **Visual Scripting** &gt; **Visual Scripting Graph**. 

3. In the new Visual Scripting window, select one of the following options: 

    * To create a new Script Graph, open the **Create new Script Graph** dropdown and select **on selected game object**.
    * To create a new State Graph, open the **Create new State Graph** dropdown and select **on selected game object**. 

4. Choose a location to save your new graph file, enter a name for the graph, and select **Save**.

    Your new graph file automatically opens in a new window. 

The new graph file should look similar to the following image: 

![A new Script Graph, created using the empty graph creation flow with starter nodes](images\vs-new-graph-starter-nodes.png)

## Next steps 

After you create your new Script Graph, attach it to a Script Machine to use it in your application. For more information, see [Attaching a graph file to a Script Machine or State Machine](vs-attach-graph-machine.md).
