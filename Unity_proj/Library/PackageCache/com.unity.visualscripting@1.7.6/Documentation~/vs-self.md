# This node

> [!NOTE]
> For versions 2019/2020 LTS, download the Visual Scripting package from the [Unity Asset Store](https://assetstore.unity.com/packages/tools/visual-bolt-163802).

The This node returns the game object that owns the machine in which the graph runs.

Generally, nodes default their target to This, so there is no need to explicitly use the This node. For example, the following Transform nodes are equivalent:

![](images/vs-this-self-node-example.png)

Not all nodes support the This inline value. Those that do not display None instead of This in their default value field; for example the Destroy node. In these cases, manually specify the connection if you mean to use This.


![](images/vs-this-self-node-example-2.png)


Use the This node in graphs even if they are not "yet" owned by a game object. The This node represents the owner of the graph at runtime, when it is used in a machine.
