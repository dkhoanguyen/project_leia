using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;
using RosCameraInfo = RosMessageTypes.Sensor.MCameraInfo;

public class RosSubscriberCameraInfo : MonoBehaviour
{
    [SerializeField] private string topicName = "";
    [SerializeField] private float maxUpdateFrequency = 100f;
    public float newDataTimeout = 1f;
    private bool isMessageProcessed = true;
    private Stack<RosCameraInfo> m_incomingMessages;
    private RosCameraInfo m_latestMessage;
    private float m_lastMessageTime = 0f;
    private bool m_firstMessageReceived = false;
    private bool m_newDataAvailable = false;

    private double[] m_camPMatrix;
    private float[] m_necessaryInfo;
    private uint m_width;
    private uint m_height;

    void Start()
    {
        // create a Stack of 1 element (but can still overflows if neccessary)
        m_incomingMessages = new Stack<RosCameraInfo>(1);
        m_necessaryInfo = new float[4];
        ROSConnection.instance.Subscribe<RosCameraInfo>(topicName, MessageCallback);
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

    public uint GetImageWidth()
    {
        return m_width;
    }

    public uint GetImageHeight()
    {
        return m_height;
    }

    public float[] GetCameraNecessaryInfo()
    {
        return m_necessaryInfo;
    }

    void MessageCallback(RosCameraInfo message)
    {
        // do nothing if the current message hasn't finished processing
        if(!isMessageProcessed)
            return;
            
        if(message != null)
        {
            if(m_firstMessageReceived)
            {
                if(Time.fixedTime - m_lastMessageTime < 1/maxUpdateFrequency)
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

        // Process Message
        m_camPMatrix = m_latestMessage.p;
        m_necessaryInfo = new float[] {(float)m_camPMatrix[2], (float)m_camPMatrix[6], (float)m_camPMatrix[0], (float)m_camPMatrix[5]};
        m_width = m_latestMessage.width;
        m_height = m_latestMessage.height;

        isMessageProcessed = true;
    }
}