using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class Timer : MonoBehaviour
{

    [SerializeField] Text Timertext;
    public static float timer;
    public static bool timeStarted = false;

    private void Update()
    {
       
            timer += Time.deltaTime;
        
            Timertext.text = Mathf.Floor(timer).ToString();

            
    }

}
