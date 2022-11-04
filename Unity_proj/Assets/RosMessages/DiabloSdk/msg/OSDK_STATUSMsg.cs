//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.DiabloSdk
{
    [Serializable]
    public class OSDK_STATUSMsg : Message
    {
        public const string k_RosMessageName = "diablo_sdk/OSDK_STATUS";
        public override string RosMessageName => k_RosMessageName;

        public byte ctrl_mode;
        public byte robot_mode;
        public uint error;
        public uint warning;

        public OSDK_STATUSMsg()
        {
            this.ctrl_mode = 0;
            this.robot_mode = 0;
            this.error = 0;
            this.warning = 0;
        }

        public OSDK_STATUSMsg(byte ctrl_mode, byte robot_mode, uint error, uint warning)
        {
            this.ctrl_mode = ctrl_mode;
            this.robot_mode = robot_mode;
            this.error = error;
            this.warning = warning;
        }

        public static OSDK_STATUSMsg Deserialize(MessageDeserializer deserializer) => new OSDK_STATUSMsg(deserializer);

        private OSDK_STATUSMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.ctrl_mode);
            deserializer.Read(out this.robot_mode);
            deserializer.Read(out this.error);
            deserializer.Read(out this.warning);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.ctrl_mode);
            serializer.Write(this.robot_mode);
            serializer.Write(this.error);
            serializer.Write(this.warning);
        }

        public override string ToString()
        {
            return "OSDK_STATUSMsg: " +
            "\nctrl_mode: " + ctrl_mode.ToString() +
            "\nrobot_mode: " + robot_mode.ToString() +
            "\nerror: " + error.ToString() +
            "\nwarning: " + warning.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
