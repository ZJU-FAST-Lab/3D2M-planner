using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Visualization;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using UnityEngine;

public class traj_render : MonoBehaviour
{

    [SerializeField]
    public string traj_render_topicname = "/scf_planner_node/traj_vis";
    
    Vector3 tp, tpflu;
    int N;
    
    // Start is called before the first frame update
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<MarkerMsg>(traj_render_topicname, markerCallback);
    }
    
    public void markerCallback(MarkerMsg marker_msg) 
    {	
    	N = marker_msg.points.Length;
    	for(int i = 0 ; i < N ; i++)
  	{
  	    tpflu.x = -(float)marker_msg.points[i].y;
  	    tpflu.y = (float)(marker_msg.points[i].z + 1.15);
  	    tpflu.z = (float)marker_msg.points[i].x;
  	    
  	    this.transform.position = tpflu;
  	}
    }

    // Update is called once per frame
    void Update()
    {


    }
    
}
