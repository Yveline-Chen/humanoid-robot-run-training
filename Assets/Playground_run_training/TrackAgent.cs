using UnityEngine;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class TrackAgent : GewuAgent
{
    //new variables for Playground_run_training project
    private WaypointManager waypointManager;
    private int currentWaypointIndex = 0;
    private int totalWaypointsReached = 0;
    private float waypointReachDistance = 5f;
    public float laneOffset = 0f;
    private ArticulationBody rootBody;
    private Vector3 startPos;
    private bool isFinishing;
    private float rawSteer;
    private float initY;
    private int maxWPReached = 0;

    public override void Initialize()
    {
        base.Initialize();
        rootBody = arts[0];
        initY = rootBody.transform.position.y; 
        startPos = rootBody.transform.position;
        waypointManager = FindObjectOfType<WaypointManager>();
        // Relay trigger events from root ArticulationBody to TrackAgent
        arts[0].gameObject.AddComponent<TriggerForwarder>().target = this;
        // Debug.Log(" ActionNum: " + ActionNum);
        // for (int i = 0; i < ActionNum; i++)
        // Debug.Log(" acts[" + i + "] = " + acts[i].name);
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        useCustomReward = true;

        // record furthest waypoint reached before reset
        if (currentWaypointIndex > maxWPReached)
        maxWPReached = currentWaypointIndex;

        // spawn: start line initially, random waypoints after reaching WP_04
        bool useRandomStart = maxWPReached >= 4;
        currentWaypointIndex = useRandomStart ? Random.Range(1, maxWPReached + 1) : 0;
        totalWaypointsReached = 0;
        isFinishing = false;
        waypointReachDistance = 5f;

        if (waypointManager != null)
        {
            Vector3 spawnPos;
            if (useRandomStart)
            spawnPos = waypointManager.GetWaypoint(currentWaypointIndex).position + waypointManager.GetWaypoint(currentWaypointIndex).right * laneOffset;
            else
            spawnPos = startPos;

            spawnPos.y = initY;
            Vector3 lookTarget = waypointManager.GetWaypoint(Mathf.Min(currentWaypointIndex + 1, waypointManager.GetWaypointCount() - 1)).position;
            lookTarget.y = spawnPos.y;
            Quaternion spawnRot = Quaternion.LookRotation(lookTarget - spawnPos, Vector3.up);
            arts[0].TeleportRoot(spawnPos, spawnRot);
            arts[0].velocity = Vector3.zero;
            arts[0].angularVelocity = Vector3.zero;
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        base.CollectObservations(sensor);

        if (waypointManager != null && !isFinishing)
        {
            for (int j = 0; j < 3; j++)
            {
                int idx = Mathf.Min(currentWaypointIndex + j, waypointManager.GetWaypointCount() - 1);
                Transform wp = waypointManager.GetWaypoint(idx);
                Vector3 target = wp.position + wp.right * laneOffset;
                Vector3 dir = arts[0].transform.InverseTransformDirection((target - rootBody.transform.position).normalized);
                sensor.AddObservation(dir);
            }
        }
        else
        {
            Vector3 finishDir = arts[0].transform.InverseTransformDirection((startPos - rootBody.transform.position).normalized);
            for (int j = 0; j < 3; j++)
            sensor.AddObservation(finishDir);
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        base.OnActionReceived(actionBuffers);

        string robotName = this.name;
        int steerIdx, leftIdx, rightIdx;
        float steerScale;
        
        if (robotName.Contains("X02Lite"))
        {
            steerIdx = 0;   
            leftIdx = 0;   
            rightIdx = 5;
            steerScale = 8f;
        }

        else if (robotName.Contains("G1"))
        {
            steerIdx = 1;   
            leftIdx = 2;   
            rightIdx = 8;
            steerScale = 20f;
        }

        else // OpenLoong
        {
            steerIdx = 1;   
            leftIdx = 1;   
            rightIdx = 7;
            steerScale = 20f;
        }

        float steer = actionBuffers.ContinuousActions[steerIdx] * steerScale;
        rawSteer = steer;

        var dL = acts[leftIdx].xDrive;
        dL.target = steer;
        acts[leftIdx].xDrive = dL;

        var dR = acts[rightIdx].xDrive;
        dR.target = -steer;
        acts[rightIdx].xDrive = dR;
    }

    void FixedUpdate()
    {   
        GewuFixedUpdate();

        // cancel yaw-swerving penalty
        AddReward(0.2f * Mathf.Abs(arts[0].angularVelocity.y));

        // alive reward
        AddReward(0.01f);

        if (waypointManager == null || rootBody == null)
        return;
          
        if (isFinishing)
        {
            Vector3 finishPos = startPos;
            finishPos.y = rootBody.transform.position.y;
            Vector3 toFinish = finishPos - rootBody.transform.position;
            float fs = Vector3.Dot(rootBody.velocity, toFinish.normalized);
            AddReward(fs * 0.1f);

            if (toFinish.magnitude < 0.5f)
            {
                AddReward(20.0f);
                EndEpisode();
            }
            return;
        }

        Transform targetWP = waypointManager.GetWaypoint(currentWaypointIndex);
        if (targetWP == null) return;

        Vector3 targetPos = targetWP.position + targetWP.right * laneOffset;

        // speed reward
        Vector3 toCurrent = (targetPos - rootBody.transform.position).normalized;
        float speed = Vector3.Dot(rootBody.velocity, toCurrent);
        AddReward(speed * 0.25f);

        //  // distance-based out-of-bounds penalty (activate when Capsule Collider is invalid)
        // float minDist = float.MaxValue;
        // int count = waypointManager.GetWaypointCount();
        // for (int i = -1; i <= 1; i++)
        // {
        //     int aIdx = (currentWaypointIndex + i + count) % count;
        //     int bIdx = (aIdx + 1) % count;
        //     Vector3 a = waypointManager.GetWaypoint(aIdx).position + waypointManager.GetWaypoint(aIdx).right * laneOffset;
        //     Vector3 b = waypointManager.GetWaypoint(bIdx).position + waypointManager.GetWaypoint(bIdx).right * laneOffset;
        //     Vector3 ab = b - a;
        //     float t = Mathf.Clamp01(Vector3.Dot(rootBody.transform.position - a, ab) / ab.sqrMagnitude);
        //     float d = Vector3.Distance(rootBody.transform.position, a + t * ab);
        //     if (d < minDist) minDist = d;
        // }
        // if (minDist > 3f)
        // {
        //     AddReward(-2.0f);
        //     EndEpisode();
        //     return;
        // }

        // curve-aware asymmetric yaw Reward
        int nxtIdx = Mathf.Min(currentWaypointIndex + 1, waypointManager.GetWaypointCount() - 1);
        Transform nxtWP = waypointManager.GetWaypoint(nxtIdx);
        Vector3 nxtPos = nxtWP.position + nxtWP.right * laneOffset;
        Vector3 toNextWP = (nxtPos - rootBody.transform.position).normalized;
        float curveAngle = Vector3.SignedAngle(toCurrent, toNextWP, Vector3.up);
        float slerp = Mathf.Clamp01(Mathf.Abs(curveAngle) / 25f) * 0.15f;
        Vector3 desiredDir = Vector3.Slerp(toCurrent, toNextWP, slerp);
        float bodyAlignment = Vector3.Dot(rootBody.transform.forward, desiredDir);
        AddReward(Mathf.Max(0, bodyAlignment) * 0.3f);

        // waypoint tracking reward
        float dist = Vector3.Distance(rootBody.transform.position, targetPos);
        if (dist < waypointReachDistance)
        {
            totalWaypointsReached++;
            if (totalWaypointsReached == 1)
            waypointReachDistance = 3f;
            if (totalWaypointsReached >= waypointManager.GetWaypointCount())
            {
                // big reward when finishing whole lap
                AddReward(20.0f);
                isFinishing = true;
                return;
            }
            else AddReward(3.0f);
            currentWaypointIndex++;
        }
    }

    // out-of-bound penalty
    public void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("OutOfBounds"))
        {
            AddReward(-2.0f);
            EndEpisode();
        }
    }
}
public class TriggerForwarder : MonoBehaviour
{
    public TrackAgent target;
    void OnTriggerEnter(Collider other) 
    {
        target.OnTriggerEnter(other); 
    }
}