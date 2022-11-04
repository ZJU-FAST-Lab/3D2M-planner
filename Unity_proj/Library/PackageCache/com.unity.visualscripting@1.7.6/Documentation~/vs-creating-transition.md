# Create a transition between states

To switch between states in a State Graph, you need to use a Script Graph called a transition.  

To create a new transition to another state in a State Graph:

1. With a State Graph open in the Graph window, do one of the following:

  - Right-click on the state where you want to make a transition, then in the context menu, select **Make Transition**.

  - Select the state where you want to make a transition, then press CTRL + click and drag away from your selected state. 

  For more information on creating a state, see [Create a new state](vs-create-state.md). 

2. Do one of the following: 

  - Select or release while on an existing state in your State Graph to connect the states with a transition. 

  - Select or release while on an empty space in the Graph Editor to automatically create a new blank Script State at the end of your transition. 

3. Select your transition node, and in the Graph Inspector, do one of the following: 

  - To use the default embedded Script Graph for your transition, leave the **Source** as **Embed**. In the **(Title)** and **(Summary)** fields, you can choose to enter a title and brief descriptive summary of your transition's Script Graph. 

  - To use an external graph file for your transition, for the **Source**, select **Graph**. In the **Graph** field, click the target icon and select a Script Graph, or click and drag a Script Graph into the **Graph** field from your Project window. 
  You can also select **New**, then enter a name and choose a save location for a new graph file. 

  > [!TIP]
  > If the Graph Inspector isn't visible in the Graph window, select **Graph Inspector** (![The Graph Inspector icon](images/vs-graph-inspector-icon.png)) from the toolbar.

4. (Optional) Double-click your new transition node to open the transition Script Graph. 

![An image of a new blank transition Script Graph open in the Graph window](images/vs-states-transition-graph-blank.png)

> [!NOTE]
> If you choose to use an embedded transition Script Graph, Visual Scripting automatically provides the Trigger Transition node you need for your graph. You can also choose to create or use an existing external graph file. For more information on choosing a graph source, see [Source types for Script Machines and State Machines](vs-graph-machine-types.md#source-types-for-script-machines-and-state-machines). 

## Create a self transition

To create a new self transition for a state in a State Graph: 

1. With a State Graph open in the Graph window, right-click on the state where you want to make the transition. 
  For more information on creating a state, see [Create a new state](vs-create-state.md).

2. In the context menu, select **Make Self Transition**. 
  Visual Scripting attaches a new Self Transition node to the state in your State Graph automatically. 

3. Select your transition node, and in the Graph Inspector, do one of the following: 

  - To use the default embedded Script Graph for your transition, leave the **Source** as **Embed**. In the **(Title)** and **(Summary)** fields, you can choose to enter a title and brief descriptive summary of your transition's Script Graph. 

  - To use an external graph file for your transition, for the **Source**, select **Graph**. In the **Graph** field, click the target icon and select a Script Graph, or click and drag a Script Graph into the **Graph** field from your Project window. 
  You can also select **New**, then enter a name and choose a save location for a new graph file. 

  > [!TIP] 
  > If the Graph Inspector isn't visible in the Graph window, select **Graph Inspector** (![The Graph Inspector icon](images/vs-graph-inspector-icon.png)) from the toolbar.

4. (Optional) Double-click your new self transition to open the transition Script Graph. 

![An image of a State Graph with a Script State node that has a self transition.](images/vs-states-self-transition.png)

> [!NOTE]
> If you choose to use an embedded transition Script Graph, Visual Scripting automatically provides the Trigger Transition node you need for your graph. You can also choose to create or use an existing external graph file. For more information on choosing a graph source, see [Source types for Script Machines and State Machines](vs-graph-machine-types.md#source-types-for-script-machines-and-state-machines) in About Script Machines and State Machines.  



