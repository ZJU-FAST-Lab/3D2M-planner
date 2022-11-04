using System.Collections;
using System.Collections.Generic;
using System;
using RosMessageTypes.UnityRoboticsDemo;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class odom_publisher : MonoBehaviour
{
    // ROS Connector
    ROSConnection m_Ros;
    
    [SerializeField]
    public string flash_topicname = "/diablo_flash";
    
    // Variables required for ROS communication
    [SerializeField]
    public string odom_topicname = "/odom";
    
    [SerializeField]
    float scale = 1.0f;
    
    Vector3<FLU> last_odom;
    double last_tick;
    bool has_odom = false;
    
    

    // Start is called before the first frame update
    void Start()
    {
         // Get ROS connection static instance
        //GameObject.Instantiate(this, new Vector3(10.0f, 1.0f,0.0f) , new Quaternion(1.0f ,0.0f ,0.0f ,0.0f));
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<OdometryMsg>(odom_topicname);
        
        ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>(flash_topicname, flashCallback);
    }
    
    public void flashCallback(PoseMsg cmd_msg) 
    {
    	//Destroy(this);  
    	//var x = cmd_msg.position.x;
    	//var y = cmd_msg.position.y;
    	//var z = cmd_msg.position.z;
    	
    	//this.transform.Translate( new Vector3((float)x,(float)y,(float)z) );
    	
    	//var new_transform = this.transform;
    	//Vector3 position    = this.transform.position.To<FLU>();
    	
    	//this.transform.position.x =  (float)x;
    	//this.transform.position.y =  (float)y;
    	//this.transform.position.z =  (float)z;
    }
    
    
    // Update is called once per frame
    void Update()
    {
   	this.Publish();
    }
    
    public void Publish()
    {
        var odom_msg = new OdometryMsg();
	var position    = this.transform.position.To<FLU>();
	position.x /= scale;
	position.y /= scale;
	position.z /= scale;
	
	var orientation = this.transform.rotation.To<FLU>();
	odom_msg.header.frame_id = "world";
	odom_msg.pose.pose.position    = position;
	//odom_msg.twist.twist.linear    = velocity;
	odom_msg.pose.pose.orientation = orientation;
        //odom_msg.pose.pose.position.x = this.transform.position.x;
        //odom_msg.pose.pose.position.y = this.transform.position.y;
        //odom_msg.pose.pose.position.z = this.transform.position.z;
        
        if( has_odom == true ){
		
	    var tick     = (DateTime.Now.ToUniversalTime().Ticks - 621355968000000000) / 10;        	
            var velocity = ( (position - last_odom) * 1000000 );
            velocity.x /= (float)( tick - last_tick ) ;
            velocity.y /= (float)( tick - last_tick ) ;
            velocity.z /= (float)( tick - last_tick ) ;
            odom_msg.twist.twist.linear    = velocity;
            //Debug.Log( Math.Sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z ) );
        }
        
        // Finally send the message to server_endpoint.py running in ROS
        m_Ros.Publish(odom_topicname, odom_msg);
        
        if( has_odom == false ) { has_odom = true; }
        last_odom = position;
        last_tick = (DateTime.Now.ToUniversalTime().Ticks - 621355968000000000) / 10; // 微秒	
        
    }

}
