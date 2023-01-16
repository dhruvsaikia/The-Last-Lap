using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class Score : MonoBehaviour
{
    
    public float m_Score = 0f;
    [SerializeField] private Text m_ScoreText;
    

    // Update is called once per frame
    void Update()
    {
        m_ScoreText.text = m_Score.ToString();
    }

    void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.tag.Equals("Obstacle")){
            m_Score+=10f;
            
        }
        if(collision.gameObject.tag.Equals("Coin")){
            Destroy(collision.gameObject);
            m_Score+=5f;
        }
        if(collision.gameObject.tag.Equals("NotTrack")){
            m_Score-=2f;
        }
    }
}
