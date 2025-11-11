using UnityEngine;

public class EndMarkerController : MonoBehaviour
{
    [Header("Enable/Disable")]
    [Tooltip("训练阶段开，正式实验关")]
    public bool enableEndMarkers = true;

    [Header("Mapping (robot meters -> world)")]
    public Vector2 originWorld = Vector2.zero;     
    public Vector2 metersToWorld = new Vector2(5f, 5f);
    public bool flipY = false;
    public bool swapXY = false;

    [Header("Refs (World-Follow Style)")]
    [Tooltip("根对象（用于显隐控制），可与 marker 相同")]
    public Transform endmarker;      // 根对象（控制显隐），可与 marker 相同
    [Tooltip("实际需要移动/显示的可视对象（例如红色X的Sprite）")]
    public Transform marker;         // 实际移动的对象
    public float markerScale = 1f;   // 单对象缩放
    void Reset()
    {
        if (!endmarker) endmarker = transform;
        if (!marker) marker = transform;
    }

    [Header("Spawned (optional)")]
    public GameObject xMarkPrefab;                 
    public Transform markerParent;
    public float prefabScale = 1f;
    private readonly System.Collections.Generic.List<GameObject> spawned = new();


    public void ShowAtRobot(double x_m, double y_m)
    {
        if (!enableEndMarkers) return;

        float x = (float)x_m, y = (float)y_m;
        if (swapXY) { (x, y) = (y, x); }
        if (flipY) y = -y;

        Vector3 worldPos = new(
            originWorld.x + x * metersToWorld.x,
            originWorld.y + y * metersToWorld.y,
            (marker ? marker.position.z : 0f)
        );

        // 单对象世界跟随：直接移动 marker，并确保 endmarker 显示
        if (marker)
        {
            marker.position = worldPos;
            if (markerScale > 0f) marker.localScale = Vector3.one * markerScale;
        }
        if (endmarker && !endmarker.gameObject.activeSelf)
            endmarker.gameObject.SetActive(true);

        // 若不使用多实例历史，直接返回
        if (xMarkPrefab == null) return;

        // 可选：若你仍想保留“生成历史痕迹”的能力，就在这里实例化
        var go = Instantiate(xMarkPrefab, worldPos, Quaternion.identity, markerParent);
        if (prefabScale > 0f) go.transform.localScale = Vector3.one * prefabScale;
        spawned.Add(go);
    }

    // 外部调用：隐藏所有❌
    public void HideAll()
    {
        if (endmarker && endmarker.gameObject.activeSelf)
            endmarker.gameObject.SetActive(false);

        for (int i = 0; i < spawned.Count; i++)
        {
            if (spawned[i]) Destroy(spawned[i]);
        }
        spawned.Clear();
    }

    // 外部调用：总开关
    public void SetEnabled(bool enabled)
    {
        enableEndMarkers = enabled;
        if (!enabled) HideAll();
    }
}