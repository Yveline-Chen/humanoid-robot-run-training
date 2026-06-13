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
    private Quaternion initRot;
    private int maxWPReached = 0;
    private Transform rightShoulder, leftShoulder, rightElbow, leftElbow;
    private Vector3 shoulderBaseR, shoulderBaseL, elbowBaseR, elbowBaseL;
    private Transform openLoongArmL, openLoongArmR;

    public override void Initialize()
    {
        base.Initialize();
        rootBody = arts[0];
        initY = rootBody.transform.position.y; 
        initRot = rootBody.transform.rotation;
        startPos = rootBody.transform.position;
        waypointManager = FindObjectOfType<WaypointManager>();
        // Relay trigger events from root ArticulationBody to TrackAgent
        arts[0].gameObject.AddComponent<TriggerForwarder>().target = this;
        // Debug.Log(" ActionNum: " + ActionNum);
        // for (int i = 0; i < ActionNum; i++)
        // Debug.Log(" acts[" + i + "] = " + acts[i].name);
        // for (int i = 0; i < arts.Length; i++)
        // Debug.Log(name + " arts[" + i + "] = " + arts[i].name);
        openLoongArmL = null; openLoongArmR = null;
        for (int i = 0; i < arts.Length; i++)
        {
            if (arts[i].name.Contains("Link_arm_l_01"))
            openLoongArmL = arts[i].transform;
            if (arts[i].name.Contains("Link_arm_r_01"))
            openLoongArmR = arts[i].transform;
        }
        rightShoulder = null; 
        leftShoulder = null; 
        rightElbow = null; 
        leftElbow = null;
        for (int i = 0; i < arts.Length; i++)
        {
            string n = arts[i].name;

            if (n.Contains("Link_arm_l_01")) 
            { 
                leftShoulder = arts[i].transform; 
                shoulderBaseL = arts[i].transform.localEulerAngles; 
            }
            if (n.Contains("Link_arm_r_01")) 
            { 
                rightShoulder = arts[i].transform; 
                shoulderBaseR = arts[i].transform.localEulerAngles; 
            }
            if (name.Contains("OpenLoong"))
            {
                if (rightShoulder != null) 
                shoulderBaseR = new Vector3(0f, 0f, -80f);
                if (leftShoulder  != null) 
                shoulderBaseL = new Vector3(0f, 0f,  80f);
            }

            if (n.Contains("shoulder_pitch") && !n.Contains("left") && !n.Contains("L_"))
            { 
                rightShoulder = arts[i].transform; 
                shoulderBaseR = arts[i].transform.localEulerAngles; 
            }
            if (n.Contains("shoulder_pitch") && (n.Contains("left") || n.Contains("L_")))
            { 
                leftShoulder = arts[i].transform; 
                shoulderBaseL = arts[i].transform.localEulerAngles; 
            }
            if ((n.Contains("elbow") || n.Contains("Elbow")) && !n.Contains("left") && !n.Contains("L_"))
            { 
                rightElbow = arts[i].transform; 
                elbowBaseR = arts[i].transform.localEulerAngles; 
            }
            if ((n.Contains("elbow") || n.Contains("Elbow")) && (n.Contains("left") || n.Contains("L_")))
            { 
                leftElbow = arts[i].transform; 
                elbowBaseL = arts[i].transform.localEulerAngles; }
        }
    }

    public override void OnEpisodeBegin()
    {
        base.OnEpisodeBegin();
        useCustomReward = true;

        // record furthest waypoint reached before reset
        if (currentWaypointIndex > maxWPReached)
        maxWPReached = currentWaypointIndex;

        // spawn: start line initially, random waypoints after reaching WP_04
        bool useRandomStart = train && maxWPReached >= 4;
        if (useRandomStart && Random.value < 0.2f)
        useRandomStart = false;
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
            Quaternion spawnRot = useRandomStart ? Quaternion.LookRotation(lookTarget - spawnPos, Vector3.up) : initRot;
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
            steerIdx = 2;   
            leftIdx = 2;   
            rightIdx = 8;
            steerScale = 20f;

            for (int i = 0; i < ActionNum; i++)
            FixStiffness(acts[i]);

            var hipR = acts[0].xDrive;
            hipR.target += -(24f * uf1 + 3f) + 12f * (1f - uf1);
            hipR.stiffness = 6000f;  
            hipR.forceLimit = 1000f;
            acts[0].xDrive = hipR;

            var hipL = acts[6].xDrive;
            hipL.target += -(24f * uf2 + 3f) + 12f * (1f - uf2);
            hipL.stiffness = 6000f;  
            hipL.forceLimit = 1000f;
            acts[6].xDrive = hipL;

            var kneeR = acts[3].xDrive;
            kneeR.target += 2f * (15f * uf1 + 20f);    
            kneeR.target += 30f * uf1 + 10f;            
            acts[3].xDrive = kneeR;

            var kneeL = acts[9].xDrive;
            kneeL.target += 2f * (15f * uf2 + 20f);
            kneeL.target += 30f * uf2 + 10f;
            acts[9].xDrive = kneeL;

            var ankR = acts[4].xDrive;
            ankR.target -= 15f * uf1 + 20f;                  
            acts[4].xDrive = ankR;

            var ankL = acts[10].xDrive;
            ankL.target -= 15f * uf2 + 20f;
            ankL.target += 10f * (1f - uf2) + 0f;
            acts[10].xDrive = ankL;
    
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
            float d = toFinish.magnitude;
            float fs = Vector3.Dot(rootBody.velocity, toFinish.normalized);
            
            if (train)
            {
                AddReward(fs * 0.1f);
                if (d < 0.5f) 
                {
                    AddReward(20.0f); 
                    EndEpisode();
                    return;
                }
            }

            if (d < 0.5f)
            {
                arts[0].velocity *= 0.7f;
                arts[0].angularVelocity *= 0.7f;

                for (int i = 0; i < ActionNum; i++)
                {
                    var dr = acts[i].xDrive;
                    dr.target *= 0.3f;
                    dr.damping = 2000f;
                    acts[i].xDrive = dr;
                }

                // freeze if tilted over
                float roll  = Mathf.Abs(rootBody.transform.eulerAngles.x);
                if (roll > 180f) 
                roll = 360f - roll;
                float pitch = Mathf.Abs(rootBody.transform.eulerAngles.z);
                if (pitch > 180f) 
                pitch = 360f - pitch;
                if (roll > 30f || pitch > 30f)
                {
                    for (int i = 0; i < ActionNum; i++)
                    {
                        var dr = acts[i].xDrive;
                        dr.target = 0f;
                        dr.stiffness = 5000f;
                        dr.damping = 3000f;
                        acts[i].xDrive = dr;
                    }
                    arts[0].velocity = Vector3.zero;
                    arts[0].angularVelocity = Vector3.zero;
                    this.enabled = false;
                    return;
                }

                if (d < 0.2f && rootBody.velocity.magnitude < 0.1f)
                {
                    for (int i = 0; i < ActionNum; i++)
                    {
                        var dr = acts[i].xDrive;
                        dr.target = 0f;
                        dr.stiffness = 8000f;
                        dr.damping = 3000f;
                        acts[i].xDrive = dr;
                    }
                    arts[0].velocity = Vector3.zero;
                    arts[0].angularVelocity = Vector3.zero;
                    this.enabled = false;
                }
                return;
            }
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

    void LateUpdate()
    {
        float amp = 40f;

        if (name.Contains("OpenLoong"))
        {
            arts[18].transform.localRotation *= Quaternion.Euler(180f, 0f, 0f);
            arts[25].transform.localRotation *= Quaternion.Euler(180f, 0f, 0f);
        }
        if (name.Contains("OpenLoong") && rightShoulder != null && leftShoulder != null)
        {
            rightShoulder.localRotation = Quaternion.Euler(shoulderBaseR) * Quaternion.Euler(amp * (uf2 - 0.5f), 0f, 0f);
            leftShoulder.localRotation = Quaternion.Euler(shoulderBaseL) * Quaternion.Euler(amp * (uf1 - 0.5f), 0f, 0f);
        }

        if (rightShoulder == null || leftShoulder == null)
        return;

        Vector3 axis = rootBody.transform.right;
        rightShoulder.rotation = Quaternion.AngleAxis(amp * (uf2 - 0.5f), axis) * rightShoulder.parent.rotation * Quaternion.Euler(shoulderBaseR);
        leftShoulder.rotation = Quaternion.AngleAxis(amp * (uf1 - 0.5f), axis) * leftShoulder.parent.rotation * Quaternion.Euler(shoulderBaseL);

        if (rightElbow != null) 
        rightElbow.localEulerAngles = elbowBaseR;
        if (leftElbow != null) 
        leftElbow.localEulerAngles = elbowBaseL;
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

    void FixStiffness(ArticulationBody joint)
    {
        var d = joint.xDrive;
        d.stiffness = 4000f;
        d.forceLimit = 800f;
        joint.xDrive = d;
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