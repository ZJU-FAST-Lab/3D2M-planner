

# Capturing player inputs using the input manager

To use nodes from the input system (input manager), the Input node must be linked from its output data port or its input trigger to a node. To use the old input system, set the **Edit** > **Project Settings** > **Player** > **Active Input Handling** to Input Manager (Old) or Both.

> [!NOTE] 
> The Input Manager (**Edit** > **Project Settings** > **Input Manager**) lists all the input types.

## To enter an input in the system

1. In a script graph that has an Event node (for example Update Event), right-click on an empty spot on the graph.</br>
   A command list appears.
2. Select **Add Node**.
   The fuzzy finder appears.
3. In the search field, enter “get axis”.
4. Select **Input: Get Axis**.</br>
   The Get Axis node appears on the graph.
5. Label the node in the **axisName** field (for example Horizontal).</br>
   > [!WARNING]
   > You must label the node with the exact spelling of the nodes listed in the Input Manager or Unity does not recognize them.</br>
   > [!TIP]
   > Copy and paste the input node name to ensure the spelling is correct.
6. Drag the output port from an event node to the input port of the Input node. Release the arrow (mouse button) so the two nodes are connected. </br>
   > [!NOTE]
   > Every time there is a frame cycle and if the data port is used, the Input node receives a signal.
7. From the **Get Axis** node, drag from the output trigger port to an input port in another node (for example a Transform node).</br>
   Every time the user clicks the key for the Get Axis node (for example, the left or right arrow), the downstream node increments.

Creating an input node using this method might not guarantee you’ve selected a node that is compatible with the selected event. Use the fuzzy finder by dragging from the event output port: only compatible nodes (that is, nodes that can be linked from that event) appear in the fuzzy finder. 


