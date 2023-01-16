using System;
using System.Collections;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof (CarController))]
    public class CarUserControl : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        [SerializeField] GameObject GameOverScreen;
        [SerializeField] GameObject m_player;

        private void Awake()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
        }


        private void FixedUpdate()
        {
            // pass the input to the car!
            float h = CrossPlatformInputManager.GetAxis("Horizontal");
            float v = CrossPlatformInputManager.GetAxis("Vertical");
#if !MOBILE_INPUT
            float handbrake = CrossPlatformInputManager.GetAxis("Jump");
            m_Car.Move(h, v, v, handbrake);
#else
            m_Car.Move(h, v, v, 0f);
#endif
        }

        private void Update()
        {
            if (m_player.transform.rotation.z == 180)
            {
                GameOver();
            }
        }

        private void GameOver()
        {
            
                StartCoroutine(WaitnWait());
                if(m_player.transform.rotation.z == 180)
                {
                    GameOverScreen.SetActive(true);
                }
            
        }

        IEnumerator WaitnWait()
        {
            yield return new WaitForSeconds(3); 
        }


    }
}
