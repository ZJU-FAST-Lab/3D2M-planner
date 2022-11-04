//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.DiabloSdk
{
    [Serializable]
    public class OSDK_LEGMOTORSMsg : Message
    {
        public const string k_RosMessageName = "diablo_sdk/OSDK_LEGMOTORS";
        public override string RosMessageName => k_RosMessageName;

        public int left_hip_enc_rev;
        public double left_hip_pos;
        public double left_hip_vel;
        public double left_hip_iq;
        public int left_knee_enc_rev;
        public double left_knee_pos;
        public double left_knee_vel;
        public double left_knee_iq;
        public int left_wheel_enc_rev;
        public double left_wheel_pos;
        public double left_wheel_vel;
        public double left_wheel_iq;
        public int right_hip_enc_rev;
        public double right_hip_pos;
        public double right_hip_vel;
        public double right_hip_iq;
        public int right_knee_enc_rev;
        public double right_knee_pos;
        public double right_knee_vel;
        public double right_knee_iq;
        public int right_wheel_enc_rev;
        public double right_wheel_pos;
        public double right_wheel_vel;
        public double right_wheel_iq;

        public OSDK_LEGMOTORSMsg()
        {
            this.left_hip_enc_rev = 0;
            this.left_hip_pos = 0.0;
            this.left_hip_vel = 0.0;
            this.left_hip_iq = 0.0;
            this.left_knee_enc_rev = 0;
            this.left_knee_pos = 0.0;
            this.left_knee_vel = 0.0;
            this.left_knee_iq = 0.0;
            this.left_wheel_enc_rev = 0;
            this.left_wheel_pos = 0.0;
            this.left_wheel_vel = 0.0;
            this.left_wheel_iq = 0.0;
            this.right_hip_enc_rev = 0;
            this.right_hip_pos = 0.0;
            this.right_hip_vel = 0.0;
            this.right_hip_iq = 0.0;
            this.right_knee_enc_rev = 0;
            this.right_knee_pos = 0.0;
            this.right_knee_vel = 0.0;
            this.right_knee_iq = 0.0;
            this.right_wheel_enc_rev = 0;
            this.right_wheel_pos = 0.0;
            this.right_wheel_vel = 0.0;
            this.right_wheel_iq = 0.0;
        }

        public OSDK_LEGMOTORSMsg(int left_hip_enc_rev, double left_hip_pos, double left_hip_vel, double left_hip_iq, int left_knee_enc_rev, double left_knee_pos, double left_knee_vel, double left_knee_iq, int left_wheel_enc_rev, double left_wheel_pos, double left_wheel_vel, double left_wheel_iq, int right_hip_enc_rev, double right_hip_pos, double right_hip_vel, double right_hip_iq, int right_knee_enc_rev, double right_knee_pos, double right_knee_vel, double right_knee_iq, int right_wheel_enc_rev, double right_wheel_pos, double right_wheel_vel, double right_wheel_iq)
        {
            this.left_hip_enc_rev = left_hip_enc_rev;
            this.left_hip_pos = left_hip_pos;
            this.left_hip_vel = left_hip_vel;
            this.left_hip_iq = left_hip_iq;
            this.left_knee_enc_rev = left_knee_enc_rev;
            this.left_knee_pos = left_knee_pos;
            this.left_knee_vel = left_knee_vel;
            this.left_knee_iq = left_knee_iq;
            this.left_wheel_enc_rev = left_wheel_enc_rev;
            this.left_wheel_pos = left_wheel_pos;
            this.left_wheel_vel = left_wheel_vel;
            this.left_wheel_iq = left_wheel_iq;
            this.right_hip_enc_rev = right_hip_enc_rev;
            this.right_hip_pos = right_hip_pos;
            this.right_hip_vel = right_hip_vel;
            this.right_hip_iq = right_hip_iq;
            this.right_knee_enc_rev = right_knee_enc_rev;
            this.right_knee_pos = right_knee_pos;
            this.right_knee_vel = right_knee_vel;
            this.right_knee_iq = right_knee_iq;
            this.right_wheel_enc_rev = right_wheel_enc_rev;
            this.right_wheel_pos = right_wheel_pos;
            this.right_wheel_vel = right_wheel_vel;
            this.right_wheel_iq = right_wheel_iq;
        }

        public static OSDK_LEGMOTORSMsg Deserialize(MessageDeserializer deserializer) => new OSDK_LEGMOTORSMsg(deserializer);

        private OSDK_LEGMOTORSMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.left_hip_enc_rev);
            deserializer.Read(out this.left_hip_pos);
            deserializer.Read(out this.left_hip_vel);
            deserializer.Read(out this.left_hip_iq);
            deserializer.Read(out this.left_knee_enc_rev);
            deserializer.Read(out this.left_knee_pos);
            deserializer.Read(out this.left_knee_vel);
            deserializer.Read(out this.left_knee_iq);
            deserializer.Read(out this.left_wheel_enc_rev);
            deserializer.Read(out this.left_wheel_pos);
            deserializer.Read(out this.left_wheel_vel);
            deserializer.Read(out this.left_wheel_iq);
            deserializer.Read(out this.right_hip_enc_rev);
            deserializer.Read(out this.right_hip_pos);
            deserializer.Read(out this.right_hip_vel);
            deserializer.Read(out this.right_hip_iq);
            deserializer.Read(out this.right_knee_enc_rev);
            deserializer.Read(out this.right_knee_pos);
            deserializer.Read(out this.right_knee_vel);
            deserializer.Read(out this.right_knee_iq);
            deserializer.Read(out this.right_wheel_enc_rev);
            deserializer.Read(out this.right_wheel_pos);
            deserializer.Read(out this.right_wheel_vel);
            deserializer.Read(out this.right_wheel_iq);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.left_hip_enc_rev);
            serializer.Write(this.left_hip_pos);
            serializer.Write(this.left_hip_vel);
            serializer.Write(this.left_hip_iq);
            serializer.Write(this.left_knee_enc_rev);
            serializer.Write(this.left_knee_pos);
            serializer.Write(this.left_knee_vel);
            serializer.Write(this.left_knee_iq);
            serializer.Write(this.left_wheel_enc_rev);
            serializer.Write(this.left_wheel_pos);
            serializer.Write(this.left_wheel_vel);
            serializer.Write(this.left_wheel_iq);
            serializer.Write(this.right_hip_enc_rev);
            serializer.Write(this.right_hip_pos);
            serializer.Write(this.right_hip_vel);
            serializer.Write(this.right_hip_iq);
            serializer.Write(this.right_knee_enc_rev);
            serializer.Write(this.right_knee_pos);
            serializer.Write(this.right_knee_vel);
            serializer.Write(this.right_knee_iq);
            serializer.Write(this.right_wheel_enc_rev);
            serializer.Write(this.right_wheel_pos);
            serializer.Write(this.right_wheel_vel);
            serializer.Write(this.right_wheel_iq);
        }

        public override string ToString()
        {
            return "OSDK_LEGMOTORSMsg: " +
            "\nleft_hip_enc_rev: " + left_hip_enc_rev.ToString() +
            "\nleft_hip_pos: " + left_hip_pos.ToString() +
            "\nleft_hip_vel: " + left_hip_vel.ToString() +
            "\nleft_hip_iq: " + left_hip_iq.ToString() +
            "\nleft_knee_enc_rev: " + left_knee_enc_rev.ToString() +
            "\nleft_knee_pos: " + left_knee_pos.ToString() +
            "\nleft_knee_vel: " + left_knee_vel.ToString() +
            "\nleft_knee_iq: " + left_knee_iq.ToString() +
            "\nleft_wheel_enc_rev: " + left_wheel_enc_rev.ToString() +
            "\nleft_wheel_pos: " + left_wheel_pos.ToString() +
            "\nleft_wheel_vel: " + left_wheel_vel.ToString() +
            "\nleft_wheel_iq: " + left_wheel_iq.ToString() +
            "\nright_hip_enc_rev: " + right_hip_enc_rev.ToString() +
            "\nright_hip_pos: " + right_hip_pos.ToString() +
            "\nright_hip_vel: " + right_hip_vel.ToString() +
            "\nright_hip_iq: " + right_hip_iq.ToString() +
            "\nright_knee_enc_rev: " + right_knee_enc_rev.ToString() +
            "\nright_knee_pos: " + right_knee_pos.ToString() +
            "\nright_knee_vel: " + right_knee_vel.ToString() +
            "\nright_knee_iq: " + right_knee_iq.ToString() +
            "\nright_wheel_enc_rev: " + right_wheel_enc_rev.ToString() +
            "\nright_wheel_pos: " + right_wheel_pos.ToString() +
            "\nright_wheel_vel: " + right_wheel_vel.ToString() +
            "\nright_wheel_iq: " + right_wheel_iq.ToString();
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
