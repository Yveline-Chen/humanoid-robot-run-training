using UnityEngine;

public class WaypointManager : MonoBehaviour
{
    public Transform[] waypoints;

    void Awake()
    {
        //adjust each waypoint oriented to the next waypoint 
        for (int i = 0; i < waypoints.Length; i++)
        {
            Transform current = waypoints[i];
            Transform next = waypoints[(i + 1) % waypoints.Length];
            current.LookAt(next);
        }
    }

    public Transform GetWaypoint(int index)
    {
        return waypoints[index % waypoints.Length];
    }

    public int GetWaypointCount()
    {
        return waypoints.Length;
    }
}

