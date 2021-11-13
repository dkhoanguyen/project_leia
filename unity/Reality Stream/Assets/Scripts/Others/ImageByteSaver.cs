using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Linq;

public class ImageByteSaver : MonoBehaviour
{
    public string filePath = "";

    public void WriteImageToFile(byte[] imageData)
    {
        File.WriteAllBytes(filePath, imageData);
    }

    public byte[] ReadImageFromFile()
    {
        return File.ReadAllBytes(filePath);
    }
}