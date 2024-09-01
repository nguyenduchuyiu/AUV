using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LIDARsensor : MonoBehaviour
{
    public int numberOfRays = 36;
    public float maxRange = 20f;
    public float angleStep = 10f;


    // Update is called once per frame
    void Update()
    {
        SimulateLIDAR();        
    }

    void SimulateLIDAR() {
        for (int i = 0; i < numberOfRays; i++) {
            // Caculate the angle for each ray
            float angle = i * angleStep;
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            Ray ray = new Ray(transform.position, direction);
            RaycastHit hit;

            //Perform the raycast
            if (Physics.Raycast(ray, out hit, maxRange)) {
                Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                Debug.Log($"Ray {i}: Distance {hit.distance}");
            } 
            else {
                
                Debug.DrawRay(transform.position, direction * maxRange, Color.green);
            }
        }
    }
}
