using UnityEngine;

    public class TrackAgent : GewuAgent
    {
        //new variables for Playground_run_training project
        private WaypointManager waypointManager;
        private int currentWaypointIndex = 0;
        private int totalWaypointsReached = 0;
        private float waypointReachDistance = 3f;
        
        public float laneOffset = 0f;

        private ArticulationBody rootBody;
        private Vector3 startPos;
        private bool isFinishing;

        public override void Initialize()
        {
            base.Initialize();
            rootBody = arts[0];
            startPos = transform.position;
            waypointManager = FindObjectOfType<WaypointManager>();
        }   

        public override void OnEpisodeBegin()
        {
            base.OnEpisodeBegin();     
            useCustomReward = true;         
            currentWaypointIndex = 0;
            totalWaypointsReached = 0;
            isFinishing = false; 
        }

        private int GetNearestWaypointIndex()
        {
            int nearestIndex = 0;
            float nearestDist = float.MaxValue;

            for (int i = 0; i < waypointManager.GetWaypointCount(); i++)
            {
                Transform wp = waypointManager.GetWaypoint(i);
                Vector3 toWP = wp.position - transform.position;
    
                if (Vector3.Dot(transform.forward, toWP) <= 0) continue;
        
                float dist = toWP.magnitude;
                
                if (dist < nearestDist)
                {
                    nearestDist = dist;
                    nearestIndex = i;
                }
            }
            return nearestIndex;
        }

        void FixedUpdate()
        {
            GewuFixedUpdate();

            // Alive Reward
            AddReward(0.01f);

            if (waypointManager == null || rootBody == null) return;
            if (isFinishing)
                {
                    Vector3 finishPos = startPos;
                    finishPos.y = transform.position.y;
                    Vector3 toFinish = finishPos - transform.position;
                    float fs = Vector3.Dot(rootBody.velocity, toFinish.normalized);
                    AddReward(fs * 0.1f);
                    AddReward(Vector3.Dot(transform.forward, toFinish.normalized) * 0.02f);

                    if (toFinish.magnitude < waypointReachDistance)
                    {
                        AddReward(20.0f);
                        EndEpisode();
                    }
                    return;
                }

            Transform targetWP = waypointManager.GetWaypoint(currentWaypointIndex);
            if (targetWP == null) return;

            Vector3 targetPos = targetWP.position + targetWP.right * laneOffset;
            Vector3 toTarget = (targetPos - transform.position).normalized;

           //Speed Reward
           float effectiveSpeed = Vector3.Dot(rootBody.velocity, toTarget);
           AddReward(effectiveSpeed * 0.1f);

           //Direction Reward
           float alignment = Vector3.Dot(transform.forward, toTarget);
           AddReward(alignment * 0.02f);
           
           //Waypoint Tracking Reward
           float dist = Vector3.Distance(transform.position, targetPos);
           if (dist < waypointReachDistance)
           {
                totalWaypointsReached++;

                if (totalWaypointsReached >= waypointManager.GetWaypointCount())
                {
                    //big reward when finishing whole lap
                    AddReward(20.0f);       
                    isFinishing = true;
                    return;
                }
                else AddReward(3.0f); 
                currentWaypointIndex++;
           }
        }
    
        //new: out-of-bound penalty
        void OnTriggerEnter(Collider other)
        {
            if (other.CompareTag("OutOfBounds"))
            {
                AddReward(-2.0f);
                EndEpisode();
            }
        }
        

    }