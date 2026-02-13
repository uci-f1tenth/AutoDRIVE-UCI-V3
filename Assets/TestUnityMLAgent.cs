using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.SceneManagement;

public class TestUnityMLAgent : Agent
{
    public override void OnEpisodeBegin()
    {
        //SceneManager.LoadScene(SceneManager.GetSceneAt(0).name);
    }

    public VehicleController VehicleController;
    public IMU InertialMeasurementUnit;
    public LIDAR LIDARUnit;
    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(VehicleController.CurrentThrottle);
        sensor.AddObservation(VehicleController.CurrentSteeringAngle);
        sensor.AddObservation(InertialMeasurementUnit.CurrentOrientationEulerAngles);
        sensor.AddObservation(InertialMeasurementUnit.CurrentAngularVelocity);
        sensor.AddObservation(InertialMeasurementUnit.CurrentLinearAcceleration);

        /*
        if (LIDARUnit.CurrentRangeArray[LIDARUnit.CurrentRangeArray.Length - 1] != null)
        {
            foreach (string lidarPoint in LIDARUnit.CurrentRangeArray)
            {
                if (lidarPoint != null)
                {
                    float lidarRange = lidarPoint.ConvertTo<float>();
                    sensor.AddObservation(lidarRange);
                }
            }
        }
        */
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (actionBuffers.ContinuousActions.Length >= 2)
        {
            VehicleController.CurrentThrottle = actionBuffers.ContinuousActions[0];
            VehicleController.CurrentSteeringAngle = actionBuffers.ContinuousActions[1];
        }
    }
}