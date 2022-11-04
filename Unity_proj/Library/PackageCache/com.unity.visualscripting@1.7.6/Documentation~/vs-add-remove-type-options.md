# Add or remove types from your Type Options

Your Type Options specify which node inputs and outputs are supported by Visual Scripting nodes. After [adding a new assembly and its nodes to your Node Library](vs-add-remove-node-library.md), you may need to add types specific to those nodes to your Type Options. Adding types to your Type Options makes them accessible in the fuzzy finder. It also allows you to assign those types to new variables in the Blackboard.

You can't use a node that has an input or output type that isn't listed in your Type Options. 

## Add a type to your Type Options 

To add a new type to your Type Options list: 

1. Go to **Edit** &gt; **Project Settings**, then select **Visual Scripting**. 
2. Expand **Type Options**. 
3. At the end of your listed types, select **Add** (+).
4. In your new type entry, select an available type from the **Type** menu. 

    Once you've made a selection, Visual Scripting adds the new type to your Type Options. To use nodes with the new type in your project, you must [regenerate your Node Library in your Project Settings](vs-configuration.md).

## Remove a type from your Type Options 

To remove a type from your Type Options list: 

1. Go to **Edit** &gt; **Project Settings**, then select **Visual Scripting**. 
2. Expand **Type Options**. 
3. In your list of Type Options, locate the entry for the type you want to remove. 
4. Select **Remove** (-).

    Visual Scripting removes the type from your Type Options. To make sure that your change appears in your project, you must [regenerate your Node Library in your Project Settings](vs-configuration.md). 