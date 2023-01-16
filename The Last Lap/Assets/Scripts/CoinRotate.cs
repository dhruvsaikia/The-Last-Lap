using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CoinRotate : MonoBehaviour
{
    

    // Update is called once per frame
    void Update()
    {
        transform.RotateAround(transform.position,Vector3.up,30*Time.deltaTime);
    }
}
