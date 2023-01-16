using System;
using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System.Collections.Generic;
#pragma warning disable 649
namespace UnityStandardAssets.Vehicles.Car
{
    internal enum CarDriveType
    {
        FrontWheelDrive,
        RearWheelDrive,
        FourWheelDrive
    }

    internal enum SpeedType
    {
        MPH,
        KPH
    }

    public class CarController : MonoBehaviour
    {
        [SerializeField] private CarDriveType m_CarDriveType = CarDriveType.FourWheelDrive;
        [SerializeField] private WheelCollider[] m_WheelColliders = new WheelCollider[4];
        [SerializeField] private GameObject[] m_WheelMeshes = new GameObject[4];
        [SerializeField] private WheelEffects[] m_WheelEffects = new WheelEffects[4];
        [SerializeField] private Vector3 m_CentreOfMassOffset;
        [SerializeField] private float m_MaximumSteerAngle;
        [Range(0, 1)] [SerializeField] private float m_SteerHelper; // 0 is raw physics , 1 the car will grip in the direction it is facing
        [Range(0, 1)] [SerializeField] private float m_TractionControl; // 0 is no traction control, 1 is full interference
        [SerializeField] private float m_FullTorqueOverAllWheels;
        [SerializeField] private float m_ReverseTorque;
        [SerializeField] private float m_MaxHandbrakeTorque;
        [SerializeField] private float m_Downforce = 100f;
        [SerializeField] private SpeedType m_SpeedType;
        [SerializeField] private float m_Topspeed = 200;
        [SerializeField] private static int NoOfGears = 5;
        [SerializeField] private float m_RevRangeBoundary = 1f;
        [SerializeField] private float m_SlipLimit;
        [SerializeField] private float m_BrakeTorque;

        private Quaternion[] m_WheelMeshLocalRotations;
        private Vector3 m_Prevpos, m_Pos;
        private float m_SteerAngle;
        private int m_GearNum;
        private float m_GearFactor;
        private float m_OldRotation;
        private float m_CurrentTorque;
        private Rigidbody m_Rigidbody;
        private const float k_ReversingThreshold = 0.01f;
        private float brakes;
        public float m_motorTorqueToIncreaseSpeed = 20;

        public float m_CurrentSpeed = 0;


        bool Skidding = false;
        public float BrakeInput { get; private set; }
        public float CurrentSteerAngle { get { return m_SteerAngle; } }
        public float CurrentSpeed { get { return m_Rigidbody.velocity.magnitude * 2.23693629f; } }
        public float MaxSpeed { get { return m_Topspeed; } }
        public float Revs { get; private set; }
        public float AccelInput { get; private set; }

        float m_lastSteer = 0;
        //[SerializeField] private float m_PowerSteerFactor = 2;
        [SerializeField] private float m_DelaySteer = 0.04f;

        Quaternion adjustQua = Quaternion.identity;
        private Coroutine m_StopSteerCoroutine;
        public float m_hardSteerStopTimer = 0.1f;
        float m_counterSteerAngle = 0;
        [SerializeField] private bool m_PowerSteering;

        [SerializeField] private Text m_GameOverScreenText;

        // Use this for initialization
        private void Start()
        {
            m_WheelMeshLocalRotations = new Quaternion[4];
            for (int i = 0; i < 4; i++)
            {
                m_WheelMeshLocalRotations[i] = m_WheelMeshes[i].transform.localRotation;
            }
            m_WheelColliders[0].attachedRigidbody.centerOfMass = m_CentreOfMassOffset;



            m_Rigidbody = GetComponent<Rigidbody>();
            m_CurrentTorque = m_FullTorqueOverAllWheels - (m_TractionControl * m_FullTorqueOverAllWheels);
        }


        private void GearChanging()
        {
            float f = Mathf.Abs(CurrentSpeed / MaxSpeed);
            float upgearlimit = (1 / (float)NoOfGears) * (m_GearNum + 1);
            float downgearlimit = (1 / (float)NoOfGears) * m_GearNum;

            if (m_GearNum > 0 && f < downgearlimit)
            {
                m_GearNum--;
            }

            if (f > upgearlimit && (m_GearNum < (NoOfGears - 1)))
            {
                m_GearNum++;
            }
        }


        // simple function to add a curved bias towards 1 for a value in the 0-1 range
        private static float CurveFactor(float factor)
        {
            return 1 - (1 - factor) * (1 - factor);
        }


        // unclamped version of Lerp, to allow value to exceed the from-to range
        private static float ULerp(float from, float to, float value)
        {
            return (1.0f - value) * from + value * to;
        }


        private void CalculateGearFactor()
        {
            float f = (1 / (float)NoOfGears);
            // gear factor is a normalised representation of the current speed within the current gear's range of speeds.
            // We smooth towards the 'target' gear factor, so that revs don't instantly snap up or down when changing gear.
            var targetGearFactor = Mathf.InverseLerp(f * m_GearNum, f * (m_GearNum + 1), Mathf.Abs(CurrentSpeed / MaxSpeed));
            m_GearFactor = Mathf.Lerp(m_GearFactor, targetGearFactor, Time.deltaTime * 5f);
        }


        private void CalculateRevs()
        {
            // calculate engine revs (for display / sound)
            // (this is done in retrospect - revs are not used in force/power calculations)
            CalculateGearFactor();
            var gearNumFactor = m_GearNum / (float)NoOfGears;
            var revsRangeMin = ULerp(0f, m_RevRangeBoundary, CurveFactor(gearNumFactor));
            var revsRangeMax = ULerp(m_RevRangeBoundary, 1f, gearNumFactor);
            Revs = ULerp(revsRangeMin, revsRangeMax, m_GearFactor);
        }


        public void Move(float steering, float accel, float footbrake, float handbrake)
        {
            for (int i = 0; i < 4; i++)
            {
                Quaternion quat;
                Vector3 position;
                m_WheelColliders[i].GetWorldPose(out position, out quat);
                m_WheelMeshes[i].transform.position = position;
                m_WheelMeshes[i].transform.rotation = quat;
            }

            //clamp input values
            steering = Mathf.Clamp(steering, -1, 1);
            //AccelInput = accel = Mathf.Clamp(accel, 0, 1); change
            AccelInput = accel = Mathf.Clamp(accel, -1, 1);
            BrakeInput = footbrake = -1 * Mathf.Clamp(footbrake, -1, 0);
            handbrake = Mathf.Clamp(handbrake, 0, 1);

            //Set the steer on the front wheels.
            //Assuming that wheels 0 and 1 are the front wheels.
            m_SteerAngle = steering * m_MaximumSteerAngle;
            m_WheelColliders[0].steerAngle = m_SteerAngle;
            m_WheelColliders[1].steerAngle = m_SteerAngle;

            SteerHelper(steering);
            ApplyDrive(accel, footbrake);
            CapSpeed();

            //Set the handbrake.
            //Assuming that wheels 2 and 3 are the rear wheels.
            if (handbrake > 0f)
            {
                var hbTorque = handbrake * m_MaxHandbrakeTorque;
                m_WheelColliders[2].brakeTorque = hbTorque;
                m_WheelColliders[3].brakeTorque = hbTorque;

            }
            else
            {
                m_WheelColliders[2].brakeTorque = 0; //change
                m_WheelColliders[3].brakeTorque = 0;

            }


            CalculateRevs();
            GearChanging();

            AddDownForce();
            //CheckForWheelSpin();
            TractionControl(accel); //change
            //CheckForCarReverse();
            HandleCyclic(m_Rigidbody);
        }

        public virtual void HandleCyclic(Rigidbody m_Rigidbody)
        {

        }



        float threshold = 0.8f;
        public bool IsUpright()
        {
            return Vector3.Dot(transform.up, Vector3.down) > threshold;
        }

        float lastUpright = 0f;
        private void Update()
        {
            if (!IsUpright())
            {
                //Debug.Log("YAY! I am upside down!");
                lastUpright = Time.time;
            }
            float timeSince = Time.time - lastUpright;
            if (timeSince >= 3.0)
            {
                //m_Rigidbody.transform.rotation = Quaternion.Euler(0, 0, 0);
                Time.timeScale =0 ; 
                m_GameOverScreenText.gameObject.SetActive(true);
            }

            brakes = 0;
            
            foreach (WheelCollider WC in m_WheelColliders)
            {
                WC.brakeTorque = brakes;
            }

        }

        private void CapSpeed()
        {
            float speed = m_Rigidbody.velocity.magnitude;
            m_CurrentSpeed = speed;
            switch (m_SpeedType)
            {
                case SpeedType.MPH:

                    speed *= 2.23693629f;
                    if (speed > m_Topspeed)
                        m_Rigidbody.velocity = (m_Topspeed / 2.23693629f) * m_Rigidbody.velocity.normalized;
                    break;

                case SpeedType.KPH:
                    speed *= 3.6f;
                    if (speed > m_Topspeed)
                        m_Rigidbody.velocity = (m_Topspeed / 3.6f) * m_Rigidbody.velocity.normalized;
                    break;
            }
        }


        private void ApplyDrive(float accel, float footbrake)
        {

            float thrustTorque;
            switch (m_CarDriveType)
            {
                case CarDriveType.FourWheelDrive:
                    thrustTorque = accel * (m_CurrentTorque / 4f);
                    for (int i = 0; i < 4; i++)
                    {

                        m_WheelColliders[i].motorTorque = thrustTorque * m_motorTorqueToIncreaseSpeed;

                    }
                    break;

                case CarDriveType.FrontWheelDrive:
                    thrustTorque = accel * (m_CurrentTorque / 2f);

                    m_WheelColliders[0].motorTorque = m_WheelColliders[1].motorTorque = thrustTorque * m_motorTorqueToIncreaseSpeed * 2; //change
                    break;

                case CarDriveType.RearWheelDrive:
                    thrustTorque = accel * (m_CurrentTorque / 2f);

                    m_WheelColliders[2].motorTorque = m_WheelColliders[3].motorTorque = thrustTorque * m_motorTorqueToIncreaseSpeed * 2; //change
                    m_WheelColliders[0].motorTorque = m_WheelColliders[1].motorTorque = thrustTorque * m_motorTorqueToIncreaseSpeed * 2; //change
                    break;

            }

            for (int i = 0; i < 4; i++)
            {
                m_WheelColliders[i].brakeTorque = 0f; //change
                if (CurrentSpeed > 5 && Vector3.Angle(transform.forward, m_Rigidbody.velocity) < 50f)
                {
                    m_WheelColliders[i].brakeTorque = m_BrakeTorque * footbrake;
                }
                else if (footbrake > 0)
                {

                    m_WheelColliders[i].motorTorque = -m_ReverseTorque * footbrake;
                }
            }
        }



        private IEnumerator StopSteer()
        {
            float time = 0;
            if (m_hardSteerStopTimer != 0)
                while (time < m_hardSteerStopTimer)
                {
                    yield return new WaitForEndOfFrame();
                    time += Time.deltaTime;
                    //m_DelaySteer = 0.04f;
                    m_Rigidbody.AddRelativeTorque(Vector3.up * (m_counterSteerAngle * Mathf.Clamp(transform.InverseTransformDirection(m_Rigidbody.velocity).z, -10f, 10f) / 400.0f) * m_SteerHelper * m_DelaySteer, ForceMode.VelocityChange);
                }
            if (m_hardSteerStopTimer == 0)
                yield return null;
            m_StopSteerCoroutine = null;
        }

        private void SteerHelper(float a_steer)
        {
            if (m_PowerSteering)
            {
                if (a_steer != 0 && m_StopSteerCoroutine != null)
                {
                    StopCoroutine(m_StopSteerCoroutine);
                    m_StopSteerCoroutine = null;
                }

                for (int i = 0; i < 4; i++)
                {
                    WheelHit wheelhit;
                    m_WheelColliders[i].GetGroundHit(out wheelhit);
                    if (wheelhit.normal == Vector3.zero)
                        return; // wheels arent on the ground so dont realign the rigidbody velocity
                }


                //get the angluar speed y axis
                float angularY = m_Rigidbody.angularVelocity.y;
                //get the radian of the speed
                float angleAngularY = angularY * Mathf.Clamp(transform.InverseTransformDirection(m_Rigidbody.velocity).z, -1f, 1f) * Mathf.Rad2Deg;
                //adjust the angle
                adjustQua = Quaternion.Lerp(adjustQua, Quaternion.AngleAxis(Mathf.Clamp(angleAngularY / 3f, -30.0f, 30.0f), Vector3.up), Time.fixedDeltaTime * 20.0f);
                Quaternion steerQua = Quaternion.Euler(new Vector3(0f, m_WheelColliders[0].steerAngle, 0f));

                float normalizer = 1.0f;
                if (steerQua.y <= adjustQua.y)
                {
                    normalizer = -1.0f;
                }
                m_counterSteerAngle = Quaternion.Angle(steerQua, adjustQua) * normalizer;
                if (a_steer == 0 && m_lastSteer != 0)
                {
                    m_StopSteerCoroutine = StartCoroutine(StopSteer());
                    // m_Rigidbody.AddRelativeTorque(Vector3.up * (angle * Mathf.Clamp(transform.InverseTransformDirection(m_Rigidbody.velocity).z, -10f, 10f) / 400.0f) * m_SteerHelper * m_PowerSteerFactor, ForceMode.VelocityChange);
                }


                // this if is needed to avoid gimbal lock problems that will make the car suddenly shift direction
                if (Mathf.Abs(m_OldRotation - transform.eulerAngles.y) < 10f)
                {
                    var turnadjust = (transform.eulerAngles.y - m_OldRotation) * m_SteerHelper;
                    Quaternion velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
                    m_Rigidbody.velocity = velRotation * m_Rigidbody.velocity;
                }


                m_OldRotation = transform.eulerAngles.y;

                m_lastSteer = a_steer;
            }
            else if (!m_PowerSteering)
            {
                for (int i = 0; i < 4; i++)
                {
                    WheelHit wheelhit;
                    m_WheelColliders[i].GetGroundHit(out wheelhit);
                    if (wheelhit.normal == Vector3.zero)
                        return; // wheels arent on the ground so dont realign the rigidbody velocity
                }

                // this if is needed to avoid gimbal lock problems that will make the car suddenly shift direction
                if (Mathf.Abs(m_OldRotation - transform.eulerAngles.y) < 10f)
                {
                    var turnadjust = (transform.eulerAngles.y - m_OldRotation) * m_SteerHelper;
                    Quaternion velRotation = Quaternion.AngleAxis(turnadjust, Vector3.up);
                    m_Rigidbody.velocity = velRotation * m_Rigidbody.velocity;
                }
                m_OldRotation = transform.eulerAngles.y;
            }


        }


        // this is used to add more grip in relation to speed
        private void AddDownForce()
        {
            m_WheelColliders[0].attachedRigidbody.AddForce(-transform.up * m_Downforce *
                                                         m_WheelColliders[0].attachedRigidbody.velocity.magnitude);
        }


        // checks if the wheels are spinning and is so does three things
        // 1) emits particles
        // 2) plays tiure skidding sounds
        // 3) leaves skidmarks on the ground
        // these effects are controlled through the WheelEffects class
        private void CheckForWheelSpin()
        {
            // loop through all wheels
            for (int i = 0; i < 4; i++)
            {
                WheelHit wheelHit;
                m_WheelColliders[i].GetGroundHit(out wheelHit);

                // is the tire slipping above the given threshhold
                if (Mathf.Abs(wheelHit.forwardSlip) >= m_SlipLimit || Mathf.Abs(wheelHit.sidewaysSlip) >= m_SlipLimit)
                {
                    m_WheelEffects[i].EmitTyreSmoke();

                }

                // end the trail generation
                m_WheelEffects[i].EndSkidTrail();
            }
        }

        // crude traction control that reduces the power to wheel if the car is wheel spinning too much
        private void TractionControl(float accel)
        {
            WheelHit wheelHit;
            switch (m_CarDriveType)
            {
                case CarDriveType.FourWheelDrive:
                    // loop through all wheels
                    for (int i = 0; i < 4; i++)
                    {
                        m_WheelColliders[i].GetGroundHit(out wheelHit);

                        AdjustTorque(wheelHit.forwardSlip, accel); //change
                    }
                    break;

                case CarDriveType.RearWheelDrive:
                    m_WheelColliders[2].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip, accel); //change

                    m_WheelColliders[3].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip, accel); //change

                    m_WheelColliders[0].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip, accel); //change

                    m_WheelColliders[1].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip, accel); //change
                    break;

                case CarDriveType.FrontWheelDrive:
                    m_WheelColliders[0].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip, accel);

                    m_WheelColliders[1].GetGroundHit(out wheelHit);
                    AdjustTorque(wheelHit.forwardSlip, accel);
                    break;
            }
        }


        private void AdjustTorque(float forwardSlip, float accel)
        {
            if (forwardSlip >= m_SlipLimit && m_CurrentTorque >= 0 && m_Rigidbody.velocity.magnitude > 20) //change
            {
                m_CurrentTorque -= 10 * m_TractionControl; //change

            }
            else
            {
                m_CurrentTorque += 10 * m_TractionControl; //change


                if (m_CurrentTorque > m_FullTorqueOverAllWheels)
                {
                    m_CurrentTorque = m_FullTorqueOverAllWheels;
                }
            }
        }


        private bool AnySkidSoundPlaying()
        {
            for (int i = 0; i < 4; i++)
            {
                if (m_WheelEffects[i].PlayingAudio)
                {
                    return true;
                }
            }
            return false;
        }

        public void StopCarOnExit(bool state)
        {
            if (state)
            {
                m_WheelColliders[0].motorTorque = 0;
                m_WheelColliders[1].motorTorque = 0;
                m_WheelColliders[2].motorTorque = 0;
                m_WheelColliders[3].motorTorque = 0;
                while (gameObject.GetComponent<Rigidbody>().velocity != Vector3.zero)
                {
                    Move(0, 0, 0, 400000000);
                }

            }


        }
    }
}
