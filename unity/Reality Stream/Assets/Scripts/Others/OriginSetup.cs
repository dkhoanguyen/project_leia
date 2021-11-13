using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Pose
{
    public string pos_x;
    public string pos_y;
    public string pos_z;
    public string rot_x;
    public string rot_y;
    public string rot_z;
    public string rot_w;
}

public class OriginSetup : MonoBehaviour
{
    [SerializeField] private RosSubscriberPoseStamped cameraPoseSubscriber;
    [SerializeField] private RosSubscriberPoseStamped headsetPoseSubscriber;
    [SerializeField] private RelativeObjectTracker headsetVRRigTracker;

    [SerializeField] private Text cameraStatusText;
    [SerializeField] private Text headsetStatusText;
    [SerializeField] private MeshRenderer trackingHeadsetMesh;

    [SerializeField] private GameObject cameraTransform;
    [SerializeField] private GameObject vrRigTransform;
    private bool trackingHeadset = false;
    private bool trackedHeadset = false;

    private void Start()
    {
        cameraStatusText.text = "Camera Pose NOT FOUND";
        headsetStatusText.text = "Headset Pose NOT FOUND";
    }

    
    private void Update()
    {
        if(!trackingHeadset && !trackedHeadset)
            if(headsetPoseSubscriber.NewDataAvailable())
                StartCoroutine("TrackHeadset");
    }

    public void SaveCameraPose()
    {
        Pose pose = TransformToPose(cameraTransform.transform);
        string poseString = JsonUtility.ToJson(pose);
        System.IO.File.WriteAllText(Application.dataPath + "/cameraPose.json", poseString);
        cameraStatusText.text = "Camera Pose SAVED TO FILE";
    }

    public void SaveHeadsetPose()
    {
        Pose pose = TransformToPose(vrRigTransform.transform);
        string poseString = JsonUtility.ToJson(pose);
        System.IO.File.WriteAllText(Application.dataPath + "/headsetPose.json", poseString);
        headsetStatusText.text = "Headset Pose SAVED TO FILE";
    }

    public void LoadCameraPose()
    {
        string fileString = Application.dataPath + "/cameraPose.json";
        string jsonString = System.IO.File.ReadAllText(fileString);
        Pose pose = JsonUtility.FromJson<Pose>(jsonString);
        Vector3 position = new Vector3(float.Parse(pose.pos_x),
                                    float.Parse(pose.pos_y),
                                    float.Parse(pose.pos_z));
        Quaternion rotation = new Quaternion(float.Parse(pose.rot_x),
                                        float.Parse(pose.rot_y),
                                        float.Parse(pose.rot_z),
                                        float.Parse(pose.rot_w));
        cameraTransform.transform.position = position;
        cameraTransform.transform.rotation = rotation;
        cameraStatusText.text = "Camera Pose LOADED FROM FILE";
    }

    public void LoadHeadsetPose()
    {
        string fileString = Application.dataPath + "/headsetPose.json";
        string jsonString = System.IO.File.ReadAllText(fileString);
        Pose pose = JsonUtility.FromJson<Pose>(jsonString);
        Vector3 position = new Vector3(float.Parse(pose.pos_x),
                                    float.Parse(pose.pos_y),
                                    float.Parse(pose.pos_z));
        Quaternion rotation = new Quaternion(float.Parse(pose.rot_x),
                                        float.Parse(pose.rot_y),
                                        float.Parse(pose.rot_z),
                                        float.Parse(pose.rot_w));
        vrRigTransform.transform.position = position;
        vrRigTransform.transform.rotation = rotation;
        headsetStatusText.text = "Headset Pose LOADED FROM FILE";
    }

    public Pose TransformToPose(Transform trans)
    {
        Pose pose = new Pose();
        pose.pos_x = trans.position.x.ToString();
        pose.pos_y = trans.position.y.ToString();
        pose.pos_z = trans.position.z.ToString();
        pose.rot_x = trans.rotation.x.ToString();
        pose.rot_y = trans.rotation.y.ToString();
        pose.rot_z = trans.rotation.z.ToString();
        pose.rot_w = trans.rotation.w.ToString();
        return pose;
    }

    IEnumerator TrackHeadset()
    {
        trackingHeadset = true;
        bool poseLocked = false;
        bool synced = false;
        float beginTime = Time.realtimeSinceStartup;

        // Wait for Pose to be constant
        while(headsetPoseSubscriber.NewDataAvailable())
        {
            if(Time.realtimeSinceStartup - beginTime > 1)
            {
                StartTrackingHeadset();
                poseLocked = true;
                break;
            }
            yield return null;
        }

        // If Pose was not constant for one second, stop Coroutine
        if(!poseLocked)
        {
            headsetStatusText.text = "Headset Pose LOST";
            trackingHeadset = false;
            StopCoroutine("TrackHeadset");
        }

        beginTime = Time.realtimeSinceStartup;
        // Wait for Pose update (2.5 seconds)
        while(headsetPoseSubscriber.NewDataAvailable())
        {
            if(Time.realtimeSinceStartup - beginTime > 2.5)
            {
                StopTrackingHeadset();
                synced = true;
                break;
            }
            yield return null;
        }

        // If Pose was not synced, stop Coroutine
        if(!synced)
        {
            headsetStatusText.text = "Headset Pose LOST";
            trackingHeadset = false;
            StopCoroutine("TrackHeadset");
        }
        else
        {
            trackingHeadset = false;
            trackedHeadset = true;
            headsetStatusText.text = "Headset Pose SYNCED";
        }

        yield return null;
    }

    public void StartTrackingCamera()
    {
        if(cameraPoseSubscriber.NewDataAvailable())
        {
            cameraStatusText.text = "Camera Pose TRACKING LIVE";
            cameraPoseSubscriber.updateObjectTransform = true;
        }
        else
        {
            cameraStatusText.text = "NO DATA FROM ROS";
        }
    }

    public void StopTrackingCamera()
    {
        cameraStatusText.text = "Camera Pose NOT TRACKING LIVE";
        cameraPoseSubscriber.updateObjectTransform = false;
    }

    public void StartTrackingHeadset()
    {
        if(headsetPoseSubscriber.NewDataAvailable())
        {
            headsetStatusText.text = "Headset Pose TRACKING LIVE";
            trackingHeadsetMesh.enabled = true;
            headsetPoseSubscriber.updateObjectTransform = true;
            headsetVRRigTracker.tracking = true;
        }
        else
        {
            headsetStatusText.text = "NO DATA FROM ROS";
        }
    }

    public void StopTrackingHeadset()
    {
        trackingHeadsetMesh.enabled = false;
        headsetPoseSubscriber.updateObjectTransform = false;
        headsetVRRigTracker.tracking = false;
    }
}