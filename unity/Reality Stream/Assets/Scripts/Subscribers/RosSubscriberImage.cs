using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;
using RosImage = RosMessageTypes.Sensor.MImage;

public class RosSubscriberImage : MonoBehaviour
{
    [SerializeField] private string topicName = "";
    [SerializeField] private float m_maxCameraFrequency = 60.0f;
    public float newDataTimeout = 1f;
    public bool displayImageToMesh = false;
    public MeshRenderer meshRenderer;
    private Texture2D texture2D;
    private byte[] imageData;
    private bool isImageProcessed = true;
    private Stack<RosImage> m_incomingImages;
    
    private float m_lastMessageTime = 0f;
    private bool m_firstMessageReceived = false;
     private bool m_newDataAvailable = false;

    void Start()
    {
        // create a Stack of 1 element (but can still overflows if neccessary)
        m_incomingImages = new Stack<RosImage>(1);
        ROSConnection.instance.Subscribe<RosImage>(topicName, ImageSubCallback);

        if(meshRenderer != null)
        {
            texture2D = new Texture2D(1, 1);
            meshRenderer.material = new Material(Shader.Find("Standard"));
            meshRenderer.material.mainTexture = texture2D;
        }
    }

    private void Update()
    {
        if (!isImageProcessed)
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

    public byte[] GetCurrentFrameData()
    {
        return imageData;
    }

    public Texture2D GetCurrentTexture2D()
    {
        return texture2D;
    }

    void ImageSubCallback(RosImage imageMessage)
    {
        // do nothing if the current frame hasn't finished processing
        if(!isImageProcessed)
            return;
            
        if(imageMessage != null)
        {
            if(m_firstMessageReceived)
            {
                if(Time.fixedTime - m_lastMessageTime < 1/m_maxCameraFrequency)
                {
                    // ignore old images from last session built up on ROS side
                    return;
                }
            }

            m_incomingImages.Push(imageMessage);

            m_firstMessageReceived = true;
            isImageProcessed = false;

            m_lastMessageTime = Time.fixedTime;
        }
    }

    private void ProcessMessage()
    {
        // if no new frames
        if(m_incomingImages.Count == 0)
            return;

        //flush all old images just in case Unity processing takes too long
        while(m_incomingImages.Count > 1)
        {
            m_incomingImages.Pop();
        }
        
        // latest frame byte[]
        imageData = m_incomingImages.Pop().data;

        
        if(displayImageToMesh)
        {
            // apply image as 2D Texture of MeshRenderer's default material
            texture2D.LoadImage(imageData);
            texture2D.Apply();
        }

        isImageProcessed = true;
    }
}