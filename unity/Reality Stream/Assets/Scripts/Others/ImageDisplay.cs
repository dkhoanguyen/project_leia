using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class ImageDisplay : MonoBehaviour
{
    public MeshRenderer meshRenderer;
    public RosSubscriberCompressedImage rosSubscriber;
    public ImageByteSaver imageSaver;
    private Texture2D texture2D;
    private byte[] imageData;

    private void Start() {
        if(meshRenderer != null)
        {
            texture2D = new Texture2D(1, 1);
            meshRenderer.material = new Material(Shader.Find("Standard"));
        }
    }

    public void GetAndSaveImage()
    {
        imageSaver.WriteImageToFile(rosSubscriber.GetCurrentFrameData());
    }

    public void DisplayImageFromFile()
    {
        imageData = imageSaver.ReadImageFromFile();
        texture2D.LoadImage(imageData);
        texture2D.Apply();
        meshRenderer.material.mainTexture = texture2D;
    }
}