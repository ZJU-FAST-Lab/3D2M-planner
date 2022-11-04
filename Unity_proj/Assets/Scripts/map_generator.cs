using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class map_generator : MonoBehaviour
{
    // Start is called before the first frame update
    Vector3 position, direction,direction2,direction3,direction4, direction5, hit_pos;
    Ray ray, ray2, ray3,ray4 ,ray5;
    RaycastHit hit;
    
    // ROS Connector
    ROSConnection m_Ros;
    
    // Variables required for ROS communication
    [SerializeField]
    string map_topicname = "/unitymap";
    uint N;
    
    [SerializeField]
    float scale_k = 20.0f;
    
    public float x_min = -10.0f;
    public float x_max = 1000.0f;
    public float y_min = -10.0f;
    public float y_max = 1000.0f;
    public float z_min = -1.0f;
    public float z_max = 40.0f;
    public float res   = 1.0f;
    
    float teck;
    
    
    void Start()
    {
     // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<Float32MultiArrayMsg>(map_topicname);
        
        position = new Vector3(0.0f, 0.0f, 0.0f);
        direction = new Vector3(0.0f, -1.0f, 0.0f);
        direction2 = new Vector3(1.0f, 0.0f, 0.0f);
        direction3 = new Vector3(0.0f, 0.0f, 1.0f);
        direction4 = new Vector3(-1.0f, 0.0f, 0.0f);
        direction5 = new Vector3(0.0f, 0.0f, -1.0f);
        ray      = new Ray(position, direction);
        ray2      = new Ray(position, direction2);
        ray3      = new Ray(position, direction3);
        ray4      = new Ray(position, direction4);
        ray5      = new Ray(position, direction5);
        
        teck  = res;
        
    }
    
    uint getN()
    {
    	uint N = 0;
    	for(float y = y_min ; y <= y_max; y += res)
    	{
    	    position.y = y ;
    	    for(float x = x_min ; x <= x_max; x += res)
    	    {
                position.x = x ;
    	        for(float z = z_min ; z <= z_max; z += res)
    		{
    		    position.z = z ;
    		    ray.origin = position;
    		    ray2.origin = position;
    		    ray3.origin = position;
    		    ray4.origin = position;
    		    ray5.origin = position;
    		    if(Physics.Raycast(ray, out hit, teck)  || Physics.Raycast(ray2, out hit, teck) || Physics.Raycast(ray3, out hit, teck) ||
    		       Physics.Raycast(ray4, out hit, teck) || Physics.Raycast(ray5, out hit, teck) )
    		    {
    		      N++;
    		    }
    		}
    	    }
    	}
    	return N;
    }
   
    
    void geneMap()
    {
 	var map_msg = new Float32MultiArrayMsg();
 	map_msg.data = new float[N*3 + 10];
 	int index = 0;
 	for(float y = y_min ; y <= y_max; y += res)
    	{
    	    position.y = y ;
    	    for(float x = x_min ; x <= x_max; x += res)
    	    {
                position.x = x ;
    	        for(float z = z_min ; z <= z_max; z += res)
    		{
    		    position.z = z ;
    		    ray.origin = position;
    		    ray2.origin = position;
    		    ray3.origin = position;
    		    ray4.origin = position;
    		    ray5.origin = position;
    		    if(Physics.Raycast(ray, out hit, teck)  || Physics.Raycast(ray2, out hit, teck) || Physics.Raycast(ray3, out hit, teck) ||
    		       Physics.Raycast(ray4, out hit, teck) || Physics.Raycast(ray5, out hit, teck) )
    		    {
    		    	//Debug.Log("p = ");
    		    	//Debug.Log(z);
    		    	//Debug.Log(-x);
    		    	//Debug.Log(y);
    		        map_msg.data[index++] =  z / scale_k;
    		        map_msg.data[index++] = -x / scale_k;
    		        map_msg.data[index++] =  y / scale_k;
    		    }
    		}
    	    }
    	}
    	
    	m_Ros.Publish(map_topicname, map_msg);
    	Debug.Log("pub");
    	Debug.Log(N*3);
    }

    // Update is called once per frame
    int times = 0;
    void Update()
    {
    	if(times > 0){N = getN();geneMap();times--;}
    }
    
    //var car = GameObject.Find("car");
    //car.transform.position = new Vector3(5.0, 25.0, 20.0);
    
    
}
