# Attach a graph file to a Script Machine or State Machine

To use a Script Graph or State Graph file in your project, you need to attach it to a Script Machine or State Machine. 

A Script Machine or State Machine is a component. Components attach to GameObjects, and help define their behavior. For more information on components and GameObjects, see [the Unity User Manual on using components](https://docs.unity3d.com/2021.1/Documentation/Manual/UsingComponents.html), or [the section on GameObjects](https://docs.unity3d.com/2021.1/Documentation/Manual/GameObjects.html).

## Add a Script Machine or State Machine component to a GameObject

1. In the Editor's Hierarchy window, select a GameObject where you'd like to add a Script Machine or State Machine. 
    If the Hierarchy window isn't visible, go to **Window** &gt; **General** &gt; **Hierarchy** or press CTRL + 4 (macOS: Cmd + 4).

2. (Optional) If the Inspector window for your selected GameObject doesn't appear, press CTRL + 3 (macOS: Cmd + 3), or go to **Window** &gt; **General** &gt; **Inspector**.  

3. In the GameObject's Inspector window, select **Add Component**. 
    The Components menu opens.

4. To add a Script Machine or State Machine, in the Components menu, go to **Visual Scripting** and select **Script Machine** or **State Machine**. Or, use the search bar to find the Script Machine or State Machine component. 

The new Script Machine or State Machine component appears in the Inspector window for your GameObject:

![A new blank Script Machine component in the Inspector window for a GameObject](images\vs-script-machine-blank.png)

## Attach a graph file to the Script Machine or State Machine 

1. In the Inspector window, locate your Script Machine or State Machine component, and ensure that you set the **Source** to **Graph**. 

2. In the **Graph** field, select the target icon and choose a compatible graph file from your project. 

You can also click and drag a file from your Project window and drop it into the **Graph** field to attach a graph to the component. For more information on how to create Script or State Graphs, see [Creating a new graph file](vs-create-graph.md).

## Next steps 

After you attach a graph to a Script Machine or State Machine, you can open your graph for editing. For more information, see [Open a graph file for editing](vs-open-graph-edit.md).
