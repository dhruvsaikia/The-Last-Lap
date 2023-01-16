using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class FinishingLine : MonoBehaviour
{
    public string[] rankList= new string[3];
    [SerializeField] private Text m_GameOver;

    [SerializeField] private Text m_Result;

    Score score;
    // Start is called before the first frame update
    void Start()
    {
        score = GetComponent<Score>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void OnTriggerEnter(Collider collider){
        for(int i= 0;i<3;i++)
        {
            rankList[i]= collider.gameObject.transform.parent.tag; 
            
        }
        m_GameOver.gameObject.SetActive(true);
        
        if(rankList[0].Equals("Player")){
            m_Result.text = "WINNER";

        }
        else
        {
            m_Result.text = "LOOSER";
        }
        Time.timeScale = 0;
    }
}
