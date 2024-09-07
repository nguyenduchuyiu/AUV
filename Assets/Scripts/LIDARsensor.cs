using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LIDARsensor : MonoBehaviour
{
    public int numberOfRays = 36;  // Number of horizontal rays
    public int verticalRays = 9;   // Number of vertical rays for incline measurement
    public float maxRange = 20f;
    public float horizontalAngleStep = 10f; // Step for horizontal rays
    public float verticalAngleStep = 5f;    // Step for vertical rays

    // Update is called once per frame
    void Update()
    {
        SimulateLIDAR();        
    }

    void SimulateLIDAR() {
        for (int i = 0; i < numberOfRays; i++) {
            float horizontalAngle = i * horizontalAngleStep;

            for (int j = -verticalRays; j <= verticalRays; j++) {
                float verticalAngle = j * verticalAngleStep;

                // Calculate the direction of the ray with both horizontal and vertical angles
                Vector3 direction = Quaternion.Euler(verticalAngle, horizontalAngle, 0) * transform.forward;

                Ray ray = new Ray(transform.position, direction);
                RaycastHit hit;

                // Perform the raycast
                if (Physics.Raycast(ray, out hit, maxRange)) {
                    // Debug.DrawRay(transform.position, direction * hit.distance, Color.red);
                    // Debug.Log($"Ray {i}-{j}: Distance {hit.distance}");
                } 
                else {
                    // Debug.DrawRay(transform.position, direction * maxRange, Color.green); 
                }
            }
        }
    }
}
