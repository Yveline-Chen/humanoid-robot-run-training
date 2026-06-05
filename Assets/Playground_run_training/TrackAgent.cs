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

        public override void Initialize()
        {
            base.Initialize();
            rootBody = GetComponentInChildren<ArticulationBody>();
            waypointManager = FindObjectOfType<WaypointManager>();
        }

        public override void OnEpisodeBegin()
        {
            base.OnEpisodeBegin();     
            useCustomReward = true;          
            currentWaypointIndex = 0;
            totalWaypointsReached = 0;
        }

        void FixedUpdate()
        {
            if (waypointManager == null || rootBody == null) return;

            Transform targetWP = waypointManager.GetWaypoint(currentWaypointIndex);
            if (targetWP == null) return;

            Vector3 targetPos = targetWP.position + targetWP.right * laneOffset;
            Vector3 toTarget = (targetPos - transform.position).normalized;

           //Speed Reward
           float effectiveSpeed = Vector3.Dot(rootBody.velocity, toTarget);
           AddReward(effectiveSpeed * 0.005f);
           
           //Direction Reward
           float alignment = Vector3.Dot(transform.forward, toTarget);
           AddReward(alignment * 0.005f);

           //Waypoint Tracking Reward
           float dist = Vector3.Distance(transform.position, targetPos);
           if (dist < waypointReachDistance)
           {
                totalWaypointsReached++;

                if (totalWaypointsReached >= waypointManager.GetWaypointCount())
                {
                    //big reward when finishing whole lap
                    AddReward(20.0f);       
                    Debug.Log($"{name} 完成一圈！");
                    EndEpisode();
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



    