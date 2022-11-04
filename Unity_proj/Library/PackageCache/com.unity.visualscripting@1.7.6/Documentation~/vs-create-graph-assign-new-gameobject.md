# Create and assign a graph to a new GameObject

You can use the empty graph creation flow to create a new graph file and assign it to a new GameObject. Visual Scripting automatically creates a new GameObject with the required components for your new graph file. 

![The Empty Graph Creation Flow window](images\vs-empty-graph-create-flow.png)

For more information on other ways to create a graph file, see [Creating a new graph file](vs-create-graph.md).

To create a new graph and assign it to a new GameObject:

1. Go to **Window** &gt; **Visual Scripting** &gt; **Visual Scripting Graph**. 

2. In the new Visual Scripting window, select one of the following options: 

    * To create a new Script Graph and GameObject, open the **Create new Script Graph** dropdown and select **on new game object**.
    * To create a new State Graph and GameObject, **Create new State Graph** dropdown and select **on new game object**.

3. Choose a location to save your new graph file, enter a name for the graph, and select **Save**. 

    > [!NOTE]
    > The GameObject you create with this method has the same name as your graph file. Once you have named and saved your graph file, the GameObject appears in the Hierarchy. 

    Your new graph file automatically opens in a new window. 

The new graph file and GameObject should look similar to the following image: 

![A new Script Graph, created using the empty graph creation flow with the On New GameObject option.](images\vs-new-graph-new-gameobject.png)

## Next steps 

After you create your new Script Graph, attach it to a Script Machine to use it in your application. For more information, see [Attaching a graph file to a Script Machine or State Machine](vs-attach-graph-machine.md).
