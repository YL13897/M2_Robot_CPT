using UnityEngine;

public class MultiDisplaySetup : MonoBehaviour
{
    public Camera mainCam;      // 主屏相机（Display 1）
    public Camera controlCam;   // 控制相机（Display 2）

    void Start()
    {
        Debug.Log("Display count: " + Display.displays.Length);
        // 启用第二块屏幕
        if (Display.displays.Length > 1)
            Display.displays[1].Activate();

        // 分配相机输出到不同的显示器
        if (mainCam) mainCam.targetDisplay = 0;
        if (controlCam) controlCam.targetDisplay = 1;
    }
}