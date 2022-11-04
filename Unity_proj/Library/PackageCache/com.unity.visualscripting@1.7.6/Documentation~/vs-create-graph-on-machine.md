# Create a graph on a Script Machine or State Machine
    
You can create a new graph file directly from a Script Machine or State Machine component on a GameObject. For more information on creating a Script Machine or State Machine, see [Attaching a graph file to a Script Machine or State Machine](vs-attach-graph-machine.md). 

## Create a new graph file from a Script Machine or State Machine

To create a new graph file from an existing Script Machine or State Machine: 

1. In the Editor's Hierarchy window, select a GameObject that has a Script Machine or State Machine. 
    If the Hierarchy window isn't visible, go to **Window** &gt; **General** &gt; **Hierarchy** or press CTRL + 4 (macOS: Cmd + 4).

2. (Optional) If the Inspector window for your selected GameObject doesn't appear, press CTRL + 3 (macOS: Cmd + 3), or go to **Window** &gt; **General** &gt; **Inspector**.

3. In the Inspector window, on your Script Machine or State Machine component with the **Source** set to **Graph**, select **New**.

4. Enter a name for your new graph file.

5. Choose a location for the file in your project, and select **Save**.

![A new Script Machine with an attached Script Graph](images\vs-script-machine.png)

## Create a new embedded graph on your Script Machine or State Machine 

You can create an embedded graph on your Script Machine or State Machine component instead of using an external graph file: 

1. In the Editor's Hierarchy window, select a GameObject that has a Script Machine or State Machine. 
    If the Hierarchy window isn't visible, go to **Window** &gt; **General** &gt; **Hierarchy** or press CTRL + 4 (macOS: Cmd + 4).

2. (Optional) If the Inspector window for your selected GameObject doesn't appear, press CTRL + 3 (macOS: Cmd + 3), or go to **Window** &gt; **General** &gt; **Inspector**.

3. In the Inspector window, on your Script Machine or State Machine component, set the **Source** to **Embed**. 

4. (Optional) In the **(Title)** field, enter a descriptive title for your embedded graph. 

5. (Optional) In the **(Summary)** field, enter a brief summary of what your embedded graph does. 

6. (Optional) To open the new embedded graph for editing, select **Edit Graph**. 

> [!NOTE]
> Unity recommends you create a graph file rather than an embedded graph. However, you might encounter some situations where an embedded graph works best. For more information on choosing the correct graph type, see [Source types for Script Machines and State Machines](vs-graph-machine-types.md#source-types-for-script-machines-and-state-machines). 

## Next steps 

After you attach a graph to a Script Machine or State Machine, you can open your graph for editing. For more information, see [Open a graph file for editing](vs-open-graph-edit.md).