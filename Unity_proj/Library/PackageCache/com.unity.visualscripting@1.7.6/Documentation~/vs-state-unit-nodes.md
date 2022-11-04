# The State Unit node

> [!NOTE]
> For versions 2019/2020 LTS, download the Visual Scripting package from the [Unity Asset Store](https://assetstore.unity.com/packages/tools/visual-bolt-163802).

**State Unit** nodes are similar to [Subgraphs](vs-subgraphs.md), however for State Graphs rather than for Script Graphs; you can nest a whole State Graph into a single node in a parent Script Graph.

The State Unit node has two control input ports to indicate when to start and stop it, and two matching control output ports to specify what to do after.

![](images/vs-state-unit-node-example.png)

When a State Unit node is started, all the start states in its nested State Graph are entered. When it is stopped, every state and transition in its nested graph is marked as inactive.
