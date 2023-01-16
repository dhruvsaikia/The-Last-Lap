using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Instructions : MonoBehaviour
{
    void Start()
 {
     //instructionsText.text = "Your text";
     Time.timeScale = 0;
 }
 
 void Update () 
 {
     if(Input.anyKeyDown)
     {
         Time.timeScale = 1;
         Destroy(gameObject);
     }
 }
}
