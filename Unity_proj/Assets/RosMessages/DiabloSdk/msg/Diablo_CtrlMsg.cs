//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.DiabloSdk
{
    [Serializable]
    public class Diablo_CtrlMsg : Message
    {
        public const string k_RosMessageName = "diablo_sdk/Diablo_Ctrl";
        public override string RosMessageName => k_RosMessageName;

        public double speed;
        public double omega;
        public double height;
        public double height_vel;
        public double roll;
        public double pitch;
        public double pitch_vel;

        public Diablo_CtrlMsg()
        {
            this.speed = 0.0;
            this.omega = 0.0;
            this.height = 0.0;
            this.height_vel = 0.0;
            this.roll = 0.0;
            this.pitch = 0.0;
            this.pitch_vel = 0.0;
        }

        public Diablo_CtrlMsg(double speed, double omega, double height, double height_vel, double roll, double pitch, double pitch_vel)
        {
            this.speed = speed;
            this.omega = omega;
            this.height = height;
            this.height_vel = height_vel;
            this.roll = roll;
            this.pitch = pitch;
            this.pitch_vel = pitch_vel;
        }

        public static Diablo_CtrlMsg Deserialize(MessageDeserializer deserializer) => new Diablo_CtrlMsg(deserializer);

        private Diablo_CtrlMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.speed);
            deserializer.Read(out this.omega);
            deserializer.Read(out this.height);
            deserializer.Read(out this.height_vel);
            deserializer.Read(out this.roll);
            deserializer.Read(out this.pitch);
            deserializer.Read(out this.pitch_vel);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.speed);
            serializer.Write(this.omega);
            serializer.Write(this.height);
            serializer.Write(this.height_vel);
            serializer.Write(this.roll);
            serializer.Write(this.pitch);
            serializer.Write(this.pitch_vel);
        }

        public override string ToString()
        {
            return "Diablo_CtrlMsg: " +
            "\nspeed: " + speed.ToString() +
            "\nomega: " + omega.ToString() +
            "\nheight: " + height.ToString() +
            "\nheight_vel: " + height_vel.ToString() +
            "\nroll: " + roll.ToString() +
            "\npitch: " + pitch.ToString() +
            "\npitch_vel: " + pitch_vel.ToString();
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