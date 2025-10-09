// M2WorldFollower2D.cs —— 把物理米映射到世界坐标（单位：米或自定比例）
using UnityEngine;

public class M2WorldFollower2D : MonoBehaviour
{
    [Header("Refs")]
    public CORC.CORCM2 m2;           // Antenna 上的 CORCM2
    public Transform marker;         // 绿色点（Sprite / GameObject）

    [Header("Mapping")]
    public Vector2 originWorld = Vector2.zero; // 物理(0,0)在世界坐标对应的位置
    public Vector2 metersToWorld = new Vector2(5f, 5f); // 1米对应多少世界单位

    [Header("Axis Options")]
    public bool flipY = false;
    public bool swapXY = false;

    [Header("Smoothing")]
    [Range(0f,1f)] public float smooth = 0.25f;
    Vector3 lastPos;

    void Reset()
    {
        marker = transform;
    }

    void Update()
    {
        if (m2 == null || !m2.IsInitialised() || marker == null) return;

        var X = m2.State["X"]; // [x,y] (m)
        float x = (float)X[0];
        float y = (float)X[1];

        if (swapXY) { var t = x; x = y; y = t; }
        if (flipY)  y = -y;

        Vector3 target = new Vector3(
            originWorld.x + x * metersToWorld.x,
            originWorld.y + y * metersToWorld.y,
            marker.position.z
        );

        if (smooth > 0f) lastPos = Vector3.Lerp(lastPos, target, 1f - Mathf.Pow(1f - smooth, Time.deltaTime * 60f));
        else             lastPos = target;

        marker.position = lastPos;
    }
    [Header("Goal Check")]
    public Vector2 goalRobot = new Vector2(0.10f, 0.05f); // 目的地(米)
    public float goalRadius = 0.01f;                      // 判定半径(米)
    public UnityEngine.Events.UnityEvent OnReachedGoal;

    void CheckGoal(float x, float y)
    {
        if (Vector2.Distance(new Vector2(x, y), goalRobot) <= goalRadius)
            OnReachedGoal?.Invoke();
    }
}