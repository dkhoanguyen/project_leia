using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RelativeObjectTracker : MonoBehaviour
{
    [SerializeField] private GameObject parentObject;
    [SerializeField] private GameObject childObject;
    [SerializeField] private GameObject matchObject;
    [SerializeField] private float updateRate = 5f;
    public bool onlyUpdateYawOrientation = true;
    public bool tracking = false;
    private float timePassed = 0f;

    void Start()
    {

    }

    void Update()
    {
        if(Time.time - timePassed > 1/updateRate && tracking)
        {
            timePassed = Time.time;

            // Setting rotation first
            Quaternion rotationDelta = Quaternion.FromToRotation(childObject.transform.forward, matchObject.transform.forward);
            if(onlyUpdateYawOrientation)
            {
                Quaternion tempParentRotation = parentObject.transform.rotation*rotationDelta;
                Vector3 parentRotation = parentObject.transform.rotation.eulerAngles;
                parentRotation.y = tempParentRotation.eulerAngles.y;
                parentObject.transform.rotation = Quaternion.Euler(parentRotation);
            }
            else
            {
                parentObject.transform.rotation = parentObject.transform.rotation*rotationDelta;
            }

            // Setting position
            Vector3 offset = matchObject.transform.position - childObject.transform.position;
            parentObject.transform.position = parentObject.transform.position + offset;
        }
    }
}