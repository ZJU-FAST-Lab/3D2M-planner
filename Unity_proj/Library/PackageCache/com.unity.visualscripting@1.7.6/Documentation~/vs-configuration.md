# Configuring your Visual Scripting project settings

You can find the following configuration options in your Visual Scripting Project Settings. To open your Project Settings, go to **Edit** &gt; **Project Settings**, then select **Visual Scripting**. 

You can find the following configuration options in your Visual Scripting Project Settings. To start using Visual Scripting in your project for the first time, you need to regenerate your Node Library, as described in the table below. 

<table>
<thead>
<tr>
<th><strong>Option</strong></th>
<th>Description</th>
</tr>
</thead>
<tbody>
<tr>
<td><strong>Type Options</strong></td>
<td>Use the Type Options list to add or remove types for your node inputs and outputs. You must regenerate your Node Library after adding or removing types. <br/>For more information on adding and removing types, see <a href="vs-add-remove-type-options.md">Add or remove types from your Type Options</a>.</td>
</tr>
<tr>
<td><strong>Node Library</strong></td>
<td>Use the Node Library to add or remove nodes in Visual Scripting, by adding or removing assemblies from your Unity Project. <br/>You might need to update your Type Options after you add new nodes to Visual Scripting, and you must regenerate your Node Library after you add or remove nodes. See <strong>Regenerate Nodes</strong>, below. <br/>For more information on adding and removing nodes from your Node Library, see <a href="vs-add-remove-node-library.md">Add or remove nodes in your Node Library</a>.</td>
</tr>
<tr>
<td><strong>Regenerate Nodes</strong></td>
<td>Select <strong>Regenerate Nodes</strong> to regenerate your Visual Scripting Node Library, then select <strong>OK</strong>. Regenerating your Node Library ensures that all your nodes are available for use in your project. <br/><div class="NOTE"><h5>NOTE</h5><p>You must regenerate your Node Library in the following circumstances: 
<ul>
<li>Before you use Visual Scripting in your project for the first time.</li>
<li>After you add or remove nodes from your Node Library.</li>
<li>After you add or remove types from your Type Options.</li>
<li>After you modify the inputs or outputs for a node.</li>
</ul></p></div></td>
</tr>
<tr>
<td><strong>Generate</strong></td>
<td>Select <strong>Generate</strong> to generate additional property provider scripts that are necessary for Unity to use custom drawers for classes and script variables inside Visual Scripting. <br/><div class="NOTE"><h5>NOTE</h5><p>To assign a default value to a custom variable type through the Unity Editor’s Inspector, you must either have access to the source code for the class, or provide a custom PropertyDrawer. For more information, see <a href="vs-custom-types.md">Custom types</a>.</p></div></td>
</tr>
<tr>
<td><strong>Create Backup</strong></td>
<td>Select <strong>Create Backup</strong> to create a new backup of your Visual Scripting graphs and settings. <br/> For more information about backups, see <a href="vs-create-restore-backups.md">Create or restore a backup</a>.</td>
</tr>
<tr>
<td><strong>Restore Backup</strong></td>
<td>Select <strong>Restore Backup</strong> to open the folder where Visual Scripting stores your backups. <br/>For more information about backups, see <a href="vs-create-restore-backups.md">Create or restore a backup</a>.</td>
</tr>
<tr>
<td><strong>Fix Missing Scripts</strong></td>
<td>Select <strong>Fix Missing Scripts</strong> to correct any issues that might occur after migrating from the Unity Asset Store version of Visual Scripting to the package version, such as any references to Visual Scripting Script Graphs and State Graphs that are missing in Script Machine or State Machine components.</td>
</tr>
</tbody>
</table>

>[!NOTE] 
> If you find that your settings don't apply after you make a change, you can [report a bug through the Unity Editor](https://unity3d.com/unity/qa/bug-reporting).
