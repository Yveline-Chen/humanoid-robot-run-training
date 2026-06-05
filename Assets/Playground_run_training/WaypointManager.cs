using UnityEngine;

public class WaypointManager : MonoBehaviour
{
    public Transform[] waypoints;

    public Transform GetWaypoint(int index)
    {
        return waypoints[index % waypoints.Length];
    }

    public int GetWaypointCount()
    {
        return waypoints.Length;
    }
}