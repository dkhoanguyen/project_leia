using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System.Collections.Generic;
using RosCompressedImage = RosMessageTypes.Sensor.MCompressedImage;
using UnityEngine.UI;

public class RosSubscriberCompressedImage : MonoBehaviour
{
    [SerializeField] private string topicName = "";
    [SerializeField] private float m_maxCameraFrequency = 60.0f;
    public float newDataTimeout = 1f;
    public bool displayImageToMesh = false;
    public MeshRenderer meshRenderer;

    private Texture2D texture2D;
    private Texture2D displayTexture2D;
    private byte[] imageData;
    private bool isImageProcessed = true;
    private Stack<RosCompressedImage> m_incomingImages;
    
    private float m_lastMessageTime = 0f;
    private bool m_firstMessageReceived = false;
    private bool m_newDataAvailable = false;

    public float displayFrameRate = 30;
    private float m_lastFrameUpdateTime = 0f;

    private bool m_initialised = false;

    void Start()
    {
        // create a Stack of 1 element (but can still overflows if neccessary)
        m_incomingImages = new Stack<RosCompressedImage>(1);
        ROSConnection.instance.Subscribe<RosCompressedImage>(topicName, ImageSubCallback);
        texture2D = new Texture2D(1, 1);
        if(meshRenderer != null)
        {
            meshRenderer.material = new Material(Shader.Find("Standard"));
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

    public void SetDisplayFrameRate(float framerate)
    {
        displayFrameRate = framerate;
    }

    public void SetDisplayFrameRate(Text framerate)
    {
        displayFrameRate = float.Parse(framerate.text);
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

    void ImageSubCallback(RosCompressedImage imageMessage)
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
        texture2D.LoadImage(imageData); // DECODES to full texture image
        texture2D.Apply();

        if(displayImageToMesh)
        {
            if(Time.time - m_lastFrameUpdateTime > 1/displayFrameRate)
            {
                m_lastFrameUpdateTime = Time.time;
                // apply image as 2D Texture of MeshRenderer's default material
                if(!m_initialised)
                {
                    displayTexture2D = new Texture2D(texture2D.width, texture2D.height);
                    m_initialised = true;
                }
                // displayTexture2D = new Texture2D(texture2D.width, texture2D.height);
                displayTexture2D.SetPixels(texture2D.GetPixels());
                displayTexture2D.Apply();
                meshRenderer.material.mainTexture = displayTexture2D;
            }
        }

        isImageProcessed = true;
    }
}