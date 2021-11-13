using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using System.Collections.Generic;
using RosPoseStamped = RosMessageTypes.Geometry.MPoseStamped;

public class RosSubscriberPoseStamped : MonoBehaviour
{
    [SerializeField] private string topicName = "";
    [SerializeField] private float m_maxUpdateFrequency = 100f;
    [SerializeField] private GameObject trackingObject;
    public bool updateObjectTransform = false;
    public float newDataTimeout = 1f;
    private bool isMessageProcessed = true;
    private Stack<RosPoseStamped> m_incomingMessages;
    private RosPoseStamped m_latestMessage;
    private float m_lastMessageTime = 0f;
    private bool m_firstMessageReceived = false;
    private bool m_newDataAvailable = false;

    void Start()
    {
        // create a Stack of 1 element (but can still overflows if neccessary)
        m_incomingMessages = new Stack<RosPoseStamped>(1);
        ROSConnection.instance.Subscribe<RosPoseStamped>(topicName, MessageCallback);
    }

    private void Update()
    {
        if (!isMessageProcessed)
            ProcessMessage();

        if(m_lastMessageTime == 0)
            m_newDataAvailable = false;
        else if(Time.fixedTime - m_lastMessageTime < newDataTimeout)
            m_newDataAvailable = true;
        else
            m_newDataAvailable = false;
    }

    public bool NewDataAvailable()
    {
        return m_newDataAvailable;
    }

    public float LatestDataTime()
    {
        return m_lastMessageTime;
    }

    void MessageCallback(RosPoseStamped message)
    {
        // do nothing if the current message hasn't finished processing
        if(!isMessageProcessed)
            return;
            
        if(message != null)
        {
            if(m_firstMessageReceived)
            {
                if(Time.fixedTime - m_lastMessageTime < 1/m_maxUpdateFrequency)
                {
                    // ignore old messages from last session built up on ROS side
                    return;
                }
            }
            m_incomingMessages.Push(message);
            m_firstMessageReceived = true;
            isMessageProcessed = false;
            m_lastMessageTime = Time.fixedTime;
        }
    }

    private void ProcessMessage()
    {
        // if no new messages
        if(m_incomingMessages.Count == 0)
            return;

        //flush all old messages just in case Unity processing takes too long
        while(m_incomingMessages.Count > 1)
        {
            m_incomingMessages.Pop();
        }
        
        // latest message
        m_latestMessage = m_incomingMessages.Pop();

        if(updateObjectTransform)
        {
            // update object's transform with Pose from message
            Vector3<FLU> position = new Vector3<FLU>((float)m_latestMessage.pose.position.x,
                                                    (float)m_latestMessage.pose.position.y,
                                                    (float)m_latestMessage.pose.position.z);
            Quaternion<FLU> rotation = new Quaternion<FLU>((float)m_latestMessage.pose.orientation.x,
                                                        (float)m_latestMessage.pose.orientation.y,
                                                        (float)m_latestMessage.pose.orientation.z,
                                                        (float)m_latestMessage.pose.orientation.w);
            trackingObject.transform.position = position.toUnity;
            trackingObject.transform.rotation = rotation.toUnity;
        }

        isMessageProcessed = true;
    }
}