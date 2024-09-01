    // Input 
    public float rollControl = 0.1f; // Sensitivity for roll control
    public float pitchControl = 0.1f; // Sensitivity for pitch control
    public float yawControl = 0.1f; // Sensitivity for yaw control
    public float thrustControl = 0.1f; // Sensitivity for thrust control
    
        // wait 0.1 sec to avoid initialization problem
        if ((startAfter -= Time.deltaTime) > 0) return;

        // Handle keyboard input to control rotor power
        if (Input.GetKeyDown(KeyCode.A))
            modifyRollRotorsRotation(-rollControl);
        if (Input.GetKeyDown(KeyCode.D))
            modifyRollRotorsRotation(rollControl);

        if (Input.GetKeyDown(KeyCode.W))
            modifyPitchRotorsRotation(pitchControl);
        if (Input.GetKeyDown(KeyCode.S))
            modifyPitchRotorsRotation(-pitchControl);

        if (Input.GetKeyDown(KeyCode.Q))
            modifyPairsRotorsRotation(-yawControl);
        if (Input.GetKeyDown(KeyCode.E))
            modifyPairsRotorsRotation(yawControl);

        if (Input.GetKeyDown(KeyCode.Space))
            modifyAllRotorsRotation(thrustControl);
        if (Input.GetKeyDown(KeyCode.LeftControl))
            modifyAllRotorsRotation(-thrustControl);

        // Apply the power adjustments from keyboard control
        pV1 = keepOnRange01(pV1);
        pV2 = keepOnRange01(pV2);
        pO1 = keepOnRange01(pO1);
        pO2 = keepOnRange01(pO2);

        helixV1.setPower(denormalizePower(pV1));
        helixV2.setPower(denormalizePower(pV2));
        helixO1.setPower(denormalizePower(pO1));
        helixO2.setPower(denormalizePower(pO2));