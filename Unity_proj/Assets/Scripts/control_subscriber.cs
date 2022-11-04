using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.DiabloSdk;
using UnityEngine;

public class control_subscriber : MonoBehaviour
{
    public GameObject[] wheels;
    public GameObject   mac;
    
    // Variables required for ROS communication
    [SerializeField]
    public string cmd_topicname = "/diablo_cmd";
    
    [SerializeField]
    double r = 0.5;
    
    [SerializeField]
    double L = 0.8;
    
    ArticulationBody lb, lb2;
    ArticulationBody rb, rb2;
    ArticulationBody macb;
    List<float> xDrive_l = new List<float>{0.0f, 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f, 0.0f };
    List<float> xDrive_r = new List<float>{0.0f, 0.0f,0.0f, 0.0f,0.0f, 0.0f,0.0f, 0.0f };
    

    //List<float> tsl = new List<float>();
    //List<float> tsr = new List<float>();
    
    
    void Start()
    {
        //ROSConnection.GetOrCreateInstance().Subscribe<Diablo_CtrlMsg>(cmd_topicname, cmdCallback);
        ROSConnection.GetOrCreateInstance().Subscribe<PoseMsg>(cmd_topicname, cmdCallback);
        
        wheels = new GameObject[4];
        wheels[0] = transform.Find("Wheel1").gameObject;
        wheels[1] = transform.Find("Wheel2").gameObject;
        wheels[2] = transform.Find("Wheel11").gameObject;
        wheels[3] = transform.Find("Wheel22").gameObject;
        mac       = transform.Find("macr").gameObject;
        lb = wheels[1].GetComponent<ArticulationBody>();
    	rb = wheels[0].GetComponent<ArticulationBody>();
        lb2 = wheels[3].GetComponent<ArticulationBody>();
    	rb2 = wheels[2].GetComponent<ArticulationBody>();
    	macb = mac.GetComponent<ArticulationBody>();
    	
    }
    
    //public void cmdCallback(Diablo_CtrlMsg cmd_msg)
    public void cmdCallback(PoseMsg cmd_msg) 
    {
    	if( lb == null || rb == null ){ return;}
    	var new_drive_l = lb.xDrive;
    	var new_drive_r = rb.xDrive;
    	var new_drive_mac = macb.xDrive;
    	
    	var v = cmd_msg.position.x;
    	var w = cmd_msg.position.y;
    	var z = cmd_msg.position.z;
    	//var v = 1.0;
    	//var w = 0.0;
    	
    	var right_w = (v + w*L) / (r);
    	var left_w  = (v - w*L) / (r);
    	
    	new_drive_l.targetVelocity = (float)(left_w  * 57.296); // 57.296 = 1rad degree
    	new_drive_r.targetVelocity = (float)(right_w * 57.296); 
    	new_drive_mac.target       = (float)(z - 1.717- 2.502);
    	
    	Debug.Log(new_drive_l.targetVelocity);
    	//xDrive_l[6] = xDrive_r[6] = (float)cmd_msg.position.x ;
    	//xDrive_l[7] = xDrive_r[7] = (float)cmd_msg.position.y ;
    	//Debug.Log(cmd_msg.position.x);
    	//Debug.Log(cmd_msg.position.y);
    /*	
    	lb.GetDriveTargetVelocities(tsl);
    	rb.GetDriveTargetVelocities(tsr);
    	
    	Debug.Log(tsl[0]);
    	Debug.Log(tsl[1]);
    	Debug.Log(tsl[2]);
    	Debug.Log(tsl[3]);
    	Debug.Log(tsl[4]);
    	Debug.Log(tsl[5]);
    	Debug.Log(tsl[6]);
    	Debug.Log(tsl[7]);
    	Debug.Log("--");
    	Debug.Log(tsr[0]);
    	Debug.Log(tsr[1]);
    	Debug.Log(tsr[2]);
    	Debug.Log(tsr[3]);
    	Debug.Log(tsr[4]);
    	Debug.Log(tsr[5]);
    	Debug.Log(tsr[6]);
    	Debug.Log(tsr[7]);
    	Debug.Log("=================");
    	*/
    	//lb.xDrive.targetVelocity = 10.0f;
    	//rb.xDrive.targetVelocity = 10.0f;
    	lb.xDrive = new_drive_l;
    	rb.xDrive = new_drive_r;
    	lb2.xDrive = new_drive_l;
    	rb2.xDrive = new_drive_r;
    	macb.xDrive = new_drive_mac;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
