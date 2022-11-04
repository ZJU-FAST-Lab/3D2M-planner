# Script Machines and State Machines

A Script Machine is a component that lets you use a Script Graph in your application. You can't use a Script Graph unless it's attached to a Script Machine. 

Script Machines exist on GameObjects. They can either link to a file that contains a Script Graph, or they can contain an embedded Script Graph file.

![An image showing an example of a Script Machine component in the Unity Inspector on a GameObject](images\vs-script-machine-blank.png)

A State Machine is the same as a Script Machine, except it contains a State Graph:

![An image showing an example of a State Machine component in the Unity Inspector on a GameObject](images\vs-state-machine-blank.png)

For more information on the difference between a Script Graph and a State Graph, see [Graphs](vs-graph-types.md).

If you add a Script Machine or State Machine component to a GameObject, Visual Scripting also automatically adds a Variables component. This Variables component holds any Object variables that you define in a Script Graph or State Graph attached to the GameObject. For more information on variables, see [Variables](vs-variables.md).

For more information on adding a Script Machine or State Machine to a GameObject and attaching a graph file, see [Attaching a graph file to a Script Machine or State Machine](vs-attach-graph-machine.md)

## Source types for Script Machines and State Machines

Script Machines and State Machines have two options for their **Source**: a graph file (**Graph**), or an embedded one (**Embed**). 

You can switch the **Source** for a Script Machine or State Machine at any time. If you switch from **Embed** to **Graph**, you'll lose your embedded graph. If you switch from **Graph** to **Embed**, your graph file still exists as a separate file from your State Machine or Script Machine inside of your project.

> [!NOTE]
> Other features of Visual Scripting, such as transitions, Super States, and Subgraphs, also have these source type options.

### The Graph source type 

Usually, you should use the **Graph** source type to make your graph faster to load and easier to maintain across your application. Any changes you make to a graph file apply to every Script Machine or State Machine that links to that graph file, even if those GameObjects don't use the same Prefab. 

If you want to use the same graph across multiple GameObjects, you should use a **Graph** source.

Unity recommends you create a graph file and use the **Graph** source type rather than an embedded graph. However, you might encounter some situations where an embedded graph works best. 

### The Embed source type 

You should consider using the **Embed** source type if: 

- You need to refer to GameObjects from your current scene in your graph and aren't using your graph on a Prefab. 
- You're using a graph on a Prefab that you plan to instantiate in your application during runtime. 
- You're only using the logic in your graph once in your application. 

You can't reuse an embedded graph across multiple GameObjects unless you're using Prefabs. An embed graph only exists on the Script Machine or State Machine where you created it. This means you can share the graph across instances of a Prefab, but not on more than one GameObject. For more information about Prefabs, see [the Prefabs section in the Unity User Manual](https://docs.unity3d.com/Manual/Prefabs.html).
