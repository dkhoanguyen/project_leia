using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using UnityEngine.UI;

public class TimeUpdater : MonoBehaviour
{
    public Text timeText;
    private TimeSpan timeFromStart;

    private void Start()
    {
        timeFromStart = TimeSpan.FromSeconds(Time.realtimeSinceStartup);
    }

    private void Update()
    {
        timeFromStart = TimeSpan.FromSeconds(Time.realtimeSinceStartup);
        string timeString = string.Format("{0:D2}:{1:D2}:{2:D2}:{3:000}", timeFromStart.Hours, timeFromStart.Minutes, timeFromStart.Seconds, timeFromStart.Milliseconds);
        timeText.text = timeString;
    }

    public void QuitGame()
    {
        Application.Quit();
    }
}
