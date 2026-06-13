using UnityEngine;

public class FollowCamera : MonoBehaviour
{
    public Transform target;         
    public Vector3 offset = new Vector3(0f, 1.5f, 6f); 
    public float smoothSpeed = 5f;

    void LateUpdate()
    {
        if (target == null) return;

        Vector3 desiredPos = target.position + target.forward * offset.z + Vector3.up * offset.y + target.right * offset.x;

        transform.position = Vector3.Lerp(transform.position, desiredPos, smoothSpeed * Time.deltaTime);
        
        transform.LookAt(target.position + Vector3.up * 0.5f);
    }
}