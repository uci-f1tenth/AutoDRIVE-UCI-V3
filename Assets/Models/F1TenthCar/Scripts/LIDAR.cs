using System.Collections;
using System.Collections.Generic;
using System.Text;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Profiling;
using Unity.VisualScripting;


public class LIDAR : MonoBehaviour
{
	/*
	This script simulates a 2D LIDAR using raycasting technique. The raycasts measure
	ranges of occluding objects with a linear range (m) between `MinimumLinearRange` and
	`MaximumLinearRange`, and an angular range (deg) between `MinimumAngularRange` and
	`MaximumAngularRange`. The `ScanRate` property determines the scanning frequency
	(Hz) while the 'Resolution' property determines the scanning resolution (deg).
	*/

	// Public data

	public bool AnimateScanner = true; // Animate scanning head of LIDAR
	public bool ShowLaserScan = false; // Visualize laser scan
	public bool ShowLidarGizmos = false; // Show LIDAR gizmos in Scene view
	public bool LogRanges = false; // Log range array to Unity Console
	public bool LogIntensities = false; // Log intensity array to Unity Console
	public GameObject Scanner; // Reference `Scanner` gameobject
	public GameObject Head; // Reference `Head` gameobject

	[Range(0, 100)] public float ScanRate = 7; // LIDAR scanning rate (Hz)
	public float MinimumLinearRange = 0.15f; // LIDAR minimum linear range (m)
	public float MaximumLinearRange = 12f; // LIDAR maximum linear range (m)
	public float MinimumAngularRange = 0; // LIDAR minimum angular range (deg)
	public float MaximumAngularRange = 359; // LIDAR maximum angular range (deg)
	public float Resolution = 1; // Angular resolution (deg)
	public int RayCastBatchSize = 32; // Number of raycasts to process in a single batch
	public float Intensity = 47.0f; // Intensity of the laser ray


	public GameObject HUD; // Use HUD to enable laser scan visualization
	public Material LaserScanMaterial; // Material for laser scan visualization
	public float LaserScanSize = 0.01f; // Size for laser scan visualization
	public Color LaserScanColor; // Color for laser scan visualization

	// Private data

	private int MeasurementsPerScan; // Measurements per scan
	private float[] RangeArray; // Array storing range values of a scan
	private float[] IntensityArray; // Array storing range values of a scan
	private float timer = 0f; // Timer to synchronize laser scan updates

	private Vector3[] laserRayDirections;
	private NativeArray<RaycastCommand> raycastCommands;
	private NativeArray<RaycastHit> raycastResults;
	private JobHandle raycastJobHandle;
    private StringBuilder rangeStringBuilder = new StringBuilder(2000);
	private int layer_mask = 1 << 0; // Mask the `Default` layer to allow raycasting only against it
	private QueryParameters raycastQueryParameters;
	private Mesh LaserScanMesh; // Mesh for laser scan visualization
	private static readonly int visualizationLayerID = 10; // `Sensor Visualization` layer

	private int[] meshIndices;
	private Vector3[] meshPoints;

	// Public getters
	public float CurrentScanRate { get { return ScanRate; } }
	public float[] CurrentRangeArray { get { return RangeArray; } }
	public float[] CurrentIntensityArray { get { return IntensityArray; } }


	private void Start()
	{
		MeasurementsPerScan = (int)((MaximumAngularRange - MinimumAngularRange) / Resolution + 1); // Compute number of measurements per scan
																								   // Debug.Log(MeasurementsPerScan);
		RangeArray = new float[MeasurementsPerScan]; // Array storing range values of a scan
		IntensityArray = new float[MeasurementsPerScan]; // Array storing range values of a scan
		
		laserRayDirections = new Vector3[MeasurementsPerScan];

		// Raycast jobs
		raycastCommands = new NativeArray<RaycastCommand>(
			MeasurementsPerScan,
			Allocator.Persistent
		);

		raycastResults = new NativeArray<RaycastHit>(
			MeasurementsPerScan,
			Allocator.Persistent
		);

		raycastQueryParameters = new QueryParameters
		{
			layerMask = layer_mask,
			hitBackfaces = false,
			hitMultipleFaces = false,
			hitTriggers = QueryTriggerInteraction.Ignore
		};

		
		// Old
		if (ShowLaserScan)
		{
			LaserScanMesh = new Mesh();
			LaserScanMesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
		}


		// visualization stuff
		meshIndices = new int[MeasurementsPerScan];
		meshPoints = new Vector3[MeasurementsPerScan];

		float angleIncrement = Resolution * Mathf.Deg2Rad;
		float halfRange = (MaximumAngularRange - MinimumAngularRange) / 2f * Mathf.Deg2Rad; // = 135°

		for (int i = 0; i < MeasurementsPerScan; i++)
		{
			float localAngle = halfRange - i * angleIncrement;

			laserRayDirections[i] = new Vector3(
				Mathf.Sin(localAngle),
				0f,
				Mathf.Cos(localAngle)
			);
		}
    }

	void FixedUpdate()
	{
		// Animate
		if (AnimateScanner) AnimateLidar();

        // In FixedUpdate, this value is always constant (e.g., 0.02)
        timer += Time.fixedDeltaTime;

        if (timer >= 1f / ScanRate)
        {
            LaserScan();
            timer -= 1f / ScanRate;
        }
    }
	
	void AnimateLidar()
	{
		Scanner.transform.Rotate(Vector3.up, Time.deltaTime * 360 * ScanRate); // Spin the scanner
	}

	void LaserScan()
	{
		Profiler.BeginSample("LIDAR.LaserScan");

		Vector3 origin = transform.position;

		// Prepare commands
		for (int i = 0; i < MeasurementsPerScan; i++)
		{
			Vector3 direction = Head.transform.rotation * laserRayDirections[i];

			raycastCommands[i] = new RaycastCommand(
				origin,
				direction,
				raycastQueryParameters,
				MaximumLinearRange
			);
		}

		// Schedule batch
		raycastJobHandle = RaycastCommand.ScheduleBatch(
			raycastCommands,
			raycastResults,
			RayCastBatchSize,
			1
		);

		// Wait for completion
		raycastJobHandle.Complete();

		// Read results
		for (int i = 0; i < MeasurementsPerScan; i++)
		{
			float d = raycastResults[i].distance;

			if (d > 0f && d > MinimumLinearRange)
				RangeArray[i] = d;
			else
				RangeArray[i] = float.PositiveInfinity;
		}

		if (ShowLaserScan && HUD.activeSelf)
			VisualizeLidarScan();

		if (LogRanges)
			LogLidarRanges();

		if (LogIntensities)
			LogLidarIntensities();

		Profiler.EndSample();
	}

	void VisualizeLidarScan()
	{
		// Old code
		// // Update indices and points based on raycast hits
		// int idx = 0;
		// for (int h = 0; h < sharedHitBuffer.Length; h++)
		// {
		// 	if (sharedHitBuffer[h].point != Vector3.zero)
		// 	{
		// 		meshIndices[idx] = idx; // Ensure sequential indices
		// 		meshPoints[idx] = sharedHitBuffer[h].point; // Append valid hit points
		// 		idx++; // Increment counter only when a valid point is found
		// 	}
		// }

		// LaserScanMesh.Clear(); // Clear the previous mesh data
		// LaserScanMesh.vertices = meshPoints; // Set new vertices
		// LaserScanMesh.SetIndices(meshIndices, MeshTopology.Points, 0); // Set indices using `Points` topology
		// LaserScanMaterial.SetFloat("_PointSize", LaserScanSize); // Set point size for material shader
		// LaserScanMaterial.SetColor("_PointColor", LaserScanColor); // Set point color for material shader

		// Graphics.DrawMesh(LaserScanMesh, Vector3.zero, Quaternion.identity, LaserScanMaterial, visualizationLayerID);
	}


	void LogLidarRanges()
	{
		// Range Array
		rangeStringBuilder.Clear();
		rangeStringBuilder.Append("Range Array: ");

		for (int i = 0; i < RangeArray.Length; i++)
		{
			rangeStringBuilder.Append(RangeArray[i]);
			rangeStringBuilder.Append(' ');
		}

		Debug.Log(rangeStringBuilder.ToString());
	}

	void LogLidarIntensities()
	{
		// Intensity Array
		StringBuilder intensityStringBuilder = new StringBuilder(2000);
		intensityStringBuilder.Append("Intensity Array: ");

		for (int i = 0; i < IntensityArray.Length; i++)
		{
			intensityStringBuilder.Append(IntensityArray[i]);
			intensityStringBuilder.Append(' ');
		}

		Debug.Log(intensityStringBuilder.ToString());
	}

	private void OnDestroy()
	{
		if (raycastCommands.IsCreated)
			raycastCommands.Dispose();

		if (raycastResults.IsCreated)
			raycastResults.Dispose();
	}
	
#if UNITY_EDITOR
	private void OnDrawGizmos()
	{
		if (!ShowLidarGizmos || laserRayDirections == null) return;

		Vector3 origin = transform.position;
		Gizmos.color = Color.red;

		for (int i = 0; i < MeasurementsPerScan; i++)
		{
			Vector3 direction = Head.transform.rotation * laserRayDirections[i];

			float distance = (RangeArray != null &&
							i < RangeArray.Length &&
							!float.IsInfinity(RangeArray[i]))
							? RangeArray[i]
							: MaximumLinearRange;

			Gizmos.DrawRay(origin, direction * distance);
		}
	}
#endif
}