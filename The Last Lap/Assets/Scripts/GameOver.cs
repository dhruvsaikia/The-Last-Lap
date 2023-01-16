using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using  UnityEngine.UI;

public class GameOver : MonoBehaviour
{
    [SerializeField] private GameObject m_player;
    [SerializeField] private Text m_GameOverText;
    private Rigidbody m_rigidbody;

    Score score;
    private void Start(){
        m_rigidbody = m_player.GetComponent<Rigidbody>();
        score = GetComponent<Score>();
    }

    private void Update(){
        if(m_rigidbody.velocity == Vector3.zero){
            StartCoroutine(Waiting());
            if(m_rigidbody.velocity == Vector3.zero){
                 
                m_GameOverText.gameObject.SetActive(true);
                Time.timeScale =0 ;

            }

        }
        
        if(score.m_Score<0){
            
            m_GameOverText.gameObject.SetActive(true);
            Time.timeScale =0 ;
        }
    }
    IEnumerator Waiting(){
        yield return new WaitForSeconds(11f);
        
    }
}
