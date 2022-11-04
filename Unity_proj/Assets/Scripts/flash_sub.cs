using System.Collections;
using System.Collections.Generic;
using System;
using RosMessageTypes.UnityRoboticsDemo;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class flash_sub : MonoBehaviour
{
    [SerializeField]
    string flash_topicname = "/diablo_flash";
    
    [SerializeField]
    string odom_topicname = "/ekf/ekf_odom";
    
    [SerializeField]
    string cmd_topicname = "/diablo_cmd_geo";
    
    
    public GameObject last_car;
    public GameObject this_car;
    public bool first;
    
    // Start is called before the first frame update
    void Start()
    {
        
    	first = true;
    	ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>(flash_topicname, flashCallback);
    }
    
    public static object GetComponentByName(GameObject targetGameObject, string componentName)
    {
	Type componentType = Type.GetType(componentName);
	object component = targetGameObject.GetComponents(componentType)[0];
	return component;
    }

    
    public void flashCallback(PoseMsg cmd_msg) 
    {
    	
    	var x = -(float)cmd_msg.position.y ;
    	var y = (float)cmd_msg.position.z;
    	var z = (float)cmd_msg.position.x;
    	
    	var ry = cmd_msg.orientation.y;
    	this_car = GameObject.Instantiate( GameObject.Find("car"), new Vector3(x,y,z) , 
    	           new Quaternion((float)cmd_msg.orientation.x ,(float)cmd_msg.orientation.y ,
    	                          (float)cmd_msg.orientation.z ,(float)cmd_msg.orientation.w));
    	//object component = GetComponentByName(this_car, "Odom_publisher");
    	//component.odom_topicname = this.odom_topicname;
    	
    	this_car.GetComponent<odom_publisher>().odom_topicname = this.odom_topicname;
    	this_car.GetComponent<control_subscriber>().cmd_topicname = this.cmd_topicname;
    	
    	
    	if (first != true){
    		Destroy(last_car);
    	}
    	
    	first = false;
    	last_car = this_car;
       
    
        Debug.Log("Generate a car");
        
    	//var new_transform = this.transform;
    	//Vector3 position    = this.transform.position.To<FLU>();
    	
    	//this.transform.position.x =  (float)x;
    	//this.transform.position.y =  (float)y;
    	//this.transform.position.z =  (float)z;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}




