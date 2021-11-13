using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;
// using OpenCVForUnity.UnityUtils;
// using OpenCVForUnity.CoreModule;
// using OpenCVForUnity.UtilsModule;
// using OpenCVForUnity.ImgcodecsModule;

public struct Point
{
    public float x;
    public float y;
    public float z;
}

public class ROSPointCloud : MonoBehaviour
{
    // Compute Shader related - GPU
    public ComputeShader m_computeShader;
    private ComputeBuffer m_computeBuffer;
    private List<Point> m_data;
    private bool m_dataInitialised = false;

    public string kernalName;

    // Render texture for GPU
    // Input Texture
    RenderTexture m_inColorRT;
    RenderTexture m_depthLowerRT;
    RenderTexture m_depthUpperRT;

    // Output Texture
    RenderTexture m_outColorRT;
    RenderTexture m_outDepthRT;

    ComputeBuffer m_inDepthLowerBuffer;
    ComputeBuffer m_inDepthUpperBuffer;

    // Compute buffers
    ComputeBuffer m_depthComputeBuffer;
    ComputeBuffer m_colorComputeBuffer;
    ComputeBuffer m_vertexIndicesBuffer;

    private int[] m_vertexIndicesData;

    // Render Material
    Material m_renderMaterial;

    Bounds bounds;

    public RawImage color_displayer;
    public RawImage depth_displayer; 

    // GPU Kernel ID
    int m_kernelHandleDepth = 0;

    // GPU Data
    private float[] m_camColorInfo;
    private float[] m_camDepthInfo;
    private float[] m_depthArray;
    private Color[] m_colorArray;
    private Vector3[] m_depthPointsArray;

    private Point[] m_pointArray;

    // Utilities
    bool m_isInitialised = false;

    // To be linked in Unity Editor
    [SerializeField] private RosSubscriberCameraInfo m_camInfoColorSub;
    [SerializeField] private RosSubscriberCameraInfo m_camInfoDepthSub;
    [SerializeField] private RosSubscriberCompressedImage m_imageColorSub;
    [SerializeField] private RosSubscriberCompressedImage m_imageDepthLowerSub;
    [SerializeField] private RosSubscriberCompressedImage m_imageDepthUpperSub;
    [SerializeField] private MeshFilter m_meshFilter;
    [SerializeField] private float targetProcessingFramerate = 60f;
    [SerializeField] private int xColorOffset = 50;
    [SerializeField] private int yColorOffset = 30;
    [SerializeField] private int rowColSkipping = 0;
    // To be generated
    private Mesh m_PCMesh;
    private Stack<Vector3> m_PCPoints;
    private Stack<int> m_PCMeshIndecies;
    private Stack<Color> m_PCMeshColours;

    // To be subscribed from ROS
    private float[] m_camInfoColor;
    private float[] m_camInfoDepth;
    private uint m_colorImageWidth, m_colorImageHeight;
    private uint m_depthImageWidth, m_depthImageHeight;
    private bool m_cameraInfoAcquired = false;
    private Texture2D m_colorTexture;
    private Texture2D m_depthLowerTexture;
    private Texture2D m_depthUpperTexture;
    private bool m_pointCloudGenerating = false;
    private bool m_pointCloudGenerated = false;

    // Test stuffs
    public Mesh mesh;
    public Material material;

    private List<GameObject> cubes_objects;

    public void SetRowColSkipping(int skipping)
    {
        rowColSkipping = skipping;
    }

    public void SetRowColSkipping(Text skipping)
    {
        rowColSkipping = int.Parse(skipping.text);
    }

    void Start()
    {
        m_PCMesh = new Mesh();
        m_PCMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        m_meshFilter.mesh = m_PCMesh;

        m_renderMaterial = material;
    }

    private void Update()
    {
        if(!m_cameraInfoAcquired)
        {
            if(m_camInfoColorSub.NewDataAvailable() && m_camInfoDepthSub.NewDataAvailable())
            {
                m_camInfoColor = m_camInfoColorSub.GetCameraNecessaryInfo();
                m_camInfoDepth = m_camInfoDepthSub.GetCameraNecessaryInfo();
                m_colorImageWidth = m_camInfoColorSub.GetImageWidth();
                m_colorImageHeight = m_camInfoColorSub.GetImageHeight();
                m_depthImageWidth = m_camInfoDepthSub.GetImageWidth();
                m_depthImageHeight = m_camInfoDepthSub.GetImageHeight();
                m_cameraInfoAcquired = true;
            }
        }
        else
        {
            // make sure to have enough images before processing
            uint numImages = 0;
            if(m_imageColorSub.NewDataAvailable())
            {
                m_colorTexture = m_imageColorSub.GetCurrentTexture2D();
                numImages++;
            }
            if(m_imageDepthLowerSub.NewDataAvailable())
            {
                m_depthLowerTexture = m_imageDepthLowerSub.GetCurrentTexture2D();
                numImages++;
            }
            if(m_imageDepthUpperSub.NewDataAvailable())
            {
                m_depthUpperTexture = m_imageDepthUpperSub.GetCurrentTexture2D();
                numImages++;
            }
            if(numImages == 3)
            {
                //// ---- Ground Truth, please do not delete this ----
                // if(m_pointCloudGenerated)
                //     UpdateMesh();
                // if(!m_pointCloudGenerating)
                // {
                //     StartCoroutine("FastGeneratePointCloud");  
                //     // StartCoroutine("GPUGeneratePointCloud");   
                // }
                // color_displayer.texture = m_colorTexture;
                // depth_displayer.texture = m_depthLowerTexture;
                GPUGeneratePointCloud();                
            }
        }
    }

    public void UpdateMesh()
    {
        m_PCMesh.Clear();
        m_PCMesh.vertices = m_PCPoints.ToArray();
        m_PCMesh.colors = m_PCMeshColours.ToArray();
        m_PCMesh.SetIndices(m_PCMeshIndecies.ToArray(), MeshTopology.Points, 0);
        m_PCMesh.uv = new Vector2[m_PCPoints.Count];
        m_PCMesh.normals = new Vector3[m_PCPoints.Count];
    }

    IEnumerator FastGeneratePointCloud()
    {
        m_pointCloudGenerating = true;
        m_pointCloudGenerated = false;
        float generationBeginTime = Time.realtimeSinceStartup;
        float lastProcessingTime = Time.realtimeSinceStartup;

        Texture2D colorTexture = m_colorTexture;
        Texture2D depthLowerTexture = m_depthLowerTexture;
        Texture2D depthUpperTexture = m_depthUpperTexture;

        Color32[] colorColor = colorTexture.GetPixels32();
        Color32[] colorDepthLower = depthLowerTexture.GetPixels32();
        Color32[] colorDepthUpper = depthUpperTexture.GetPixels32();

        m_PCPoints = new Stack<Vector3>();
        m_PCMeshIndecies = new Stack<int>();
        m_PCMeshColours = new Stack<Color>();

        int depthHeight = depthLowerTexture.height;
        int depthWidth = depthLowerTexture.width;
        int colorHeight = colorTexture.height;
        int colorWidth = colorTexture.width;

        int row = 0, col = 0;
        for(row = 0; row < depthHeight; row = row + rowColSkipping + 1)
        {
            for(col = 0; col < depthWidth; col = col + rowColSkipping + 1)
            {
                if(Time.realtimeSinceStartup - lastProcessingTime < 1/targetProcessingFramerate)
                {
                    // Using the green (g) channel of Color, because all r, g, b give the same value (grey)
                    float pixelDepth = ((float)colorDepthLower[row*depthWidth + col].g + (float)colorDepthUpper[row*depthWidth + col].g*256f)*0.001f;

                    if(pixelDepth >  0)
                    {
                        // Coordinate
                        // NEEDS TO BE LEFT HAND RULES
                        float x = (float)((col - m_camInfoDepth[0]) / m_camInfoDepth[2] * pixelDepth); 
                        float y = (float)((row - m_camInfoDepth[1]) / m_camInfoDepth[3] * pixelDepth); 
                        float z = (float)pixelDepth;
                        m_PCPoints.Push(new Vector3 (x, y, z));
                        m_PCMeshIndecies.Push(m_PCPoints.Count - 1);

                        // // Colour
                        int colColor = (int)((x / z * m_camInfoColor[2]) + m_camInfoColor[0]) + xColorOffset;
                        int rowColor = (int)((y / z * m_camInfoColor[3]) + m_camInfoColor[1]) + yColorOffset;

                        Color color = Color.clear;
                        if(colColor >= 0 && colColor < colorWidth && rowColor >= 0 && rowColor < colorHeight)
                        {
                            color = colorColor[rowColor*colorWidth + colColor];
                        }
                        m_PCMeshColours.Push(color);
                    }
                }
                else
                {
                    yield return null;
                    lastProcessingTime = Time.realtimeSinceStartup;
                }
            }
        }

        // can print this out to debug efficiency
        float processTime = Time.realtimeSinceStartup - generationBeginTime;
        m_pointCloudGenerating = false;
        m_pointCloudGenerated = true;
        yield return null;
    }

    void GPUGeneratePointCloud()
    {
        m_pointCloudGenerating = true;
        m_pointCloudGenerated = false;

        float[] test_depth  = new float[(int)m_colorImageWidth * (int)m_colorImageHeight];

        if(!m_isInitialised)
        {
            m_outColorRT = new RenderTexture((int)m_colorImageWidth, (int)m_colorImageHeight, 32);
            m_outColorRT.enableRandomWrite = true;
            m_outColorRT.Create();

            m_outDepthRT = new RenderTexture((int)m_colorImageWidth, (int)m_colorImageHeight, 32);
            m_outDepthRT.enableRandomWrite = true;
            m_outDepthRT.Create();

            m_depthArray = new float[(int)m_colorImageWidth * (int)m_colorImageHeight];
            m_camColorInfo = new float[4];
            m_camDepthInfo = new float[4];

            m_colorArray = new Color[(int)m_colorImageWidth * (int)m_colorImageHeight];
            m_depthPointsArray = new Vector3[(int)m_colorImageWidth * (int)m_colorImageHeight];

            m_vertexIndicesData = new int[(int)m_colorImageWidth * (int)m_colorImageHeight];

            m_kernelHandleDepth = m_computeShader.FindKernel(kernalName);
            m_depthComputeBuffer = new ComputeBuffer(m_depthPointsArray.Length, sizeof(float)*3);
            m_colorComputeBuffer = new ComputeBuffer(m_depthPointsArray.Length, sizeof(float)*4);

            m_isInitialised = true;
        }

        // Depth rendering  
        m_depthComputeBuffer.SetData(m_depthPointsArray);
        // Color rendering
        m_colorComputeBuffer.SetData(m_colorArray);


        // Camera transform
        Matrix4x4 cam_transform = gameObject.transform.localToWorldMatrix;

        // Set inputs for GPU kernels
        m_computeShader.SetTexture(m_kernelHandleDepth,"inColor",m_colorTexture);
        m_computeShader.SetTexture(m_kernelHandleDepth,"inDepthLower",m_depthLowerTexture);
        m_computeShader.SetTexture(m_kernelHandleDepth,"inDepthUpper",m_depthUpperTexture);

        m_computeShader.SetFloats("camInfoColor",m_camInfoColor);
        m_computeShader.SetFloats("camInfoDepth",m_camInfoDepth);

        m_computeShader.SetInt("xColorOffset",xColorOffset);
        m_computeShader.SetInt("yColorOffset",yColorOffset);
        
        m_computeShader.SetInt("colorImageWidth",(int)m_colorImageWidth);
        m_computeShader.SetInt("colorImageHeight",(int)m_colorImageHeight);

        // Set outputs for GPU kernels
        m_computeShader.SetBuffer(m_kernelHandleDepth,"depthOut",m_depthComputeBuffer);
        m_computeShader.SetBuffer(m_kernelHandleDepth,"colorOut",m_colorComputeBuffer);

        // Set values to Material for rendering
        m_renderMaterial.SetBuffer("vertexPosition",m_depthComputeBuffer);
        m_renderMaterial.SetBuffer("vertexColor",m_colorComputeBuffer);
        m_renderMaterial.SetTexture("colorTexture",m_colorTexture);

        // Set camera transform
        m_computeShader.SetMatrix("cameraTransform",cam_transform);

        bounds = new Bounds(Vector3.zero, Vector3.one * 1000);

        // Dispatch to invoke GPU computing
        m_computeShader.Dispatch(m_kernelHandleDepth,640/32,480/32,1);
        // m_depthComputeBuffer.GetData(m_depthPointsArray);
        // Debug.Log(m_depthPointsArray[0][2]);

        Graphics.DrawProcedural(m_renderMaterial, bounds, MeshTopology.Points, m_vertexIndicesData.Length, 1);
    
        m_pointCloudGenerating = false;
        m_pointCloudGenerated = true;
    }
}