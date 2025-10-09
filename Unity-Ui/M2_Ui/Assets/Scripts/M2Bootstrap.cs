// M2Bootstrap.cs (with auto-assign + optional auto-reconnect)
using UnityEngine;
using System.Collections; // for IEnumerator/Coroutine

namespace CORC.Demo
{
    [RequireComponent(typeof(CORC.CORCM2))]
    public class M2Bootstrap : MonoBehaviour
    {
        [Header("Server")]
        public string serverIp = "192.168.8.104";
        public int serverPort = 2048;

        [Header("Connection")]
        public bool autoReconnect = true;        // 若启动时未连上，是否自动重连
        public float reconnectInterval = 2f;     // 重连间隔（秒）

        [Header("Logging (optional)")]
        public bool enableCsvLogging = false;

        [Tooltip("如不填则默认 persistentDataPath/m2_log.csv")]
        public string csvPathOverride = "";

        [Header("Refs")]
        public CORC.CORCM2 m2;   // 在同一物体上或场景中挂一个 CORCM2 组件

        private string csvPath;
        private Coroutine reconnectCo;

        void Awake()
        {
            // 自动获取同物体上的 CORCM2，避免漏拖引用
            if (m2 == null)
                m2 = GetComponent<CORC.CORCM2>();
        }

        void Start()
        {
            if (m2 == null)
            {
                Debug.LogError("[M2Bootstrap] Please assign CORCM2 component.");
                return;
            }

            // 先尝试连接一次
            TryConnectOnce();

            // 可选：自动重连（例如机器人端稍后才启动时）
            if (autoReconnect)
                reconnectCo = StartCoroutine(AutoReconnectLoop());
        }

        void TryConnectOnce()
        {
            m2.Init(serverIp, serverPort);

            if (enableCsvLogging)
            {
                csvPath = string.IsNullOrEmpty(csvPathOverride)
                    ? System.IO.Path.Combine(Application.persistentDataPath, "m2_log.csv")
                    : csvPathOverride;

                m2.SetLoggingFile(csvPath);
                m2.SetLogging(true);
                Debug.Log($"[M2Bootstrap] Logging to: {csvPath}");
            }
        }

        IEnumerator AutoReconnectLoop()
        {
            // 如果尚未初始化成功，则间隔重试
            while (m2 != null && !m2.IsInitialised())
            {
                Debug.Log("[M2Bootstrap] Not connected yet, retrying...");
                TryConnectOnce();
                yield return new WaitForSeconds(reconnectInterval);
            }

            if (m2 != null && m2.IsInitialised())
                Debug.Log("[M2Bootstrap] Connected.");
        }

        void OnApplicationQuit()
        {
            if (reconnectCo != null) StopCoroutine(reconnectCo);

            if (m2 != null)
            {
                if (enableCsvLogging) m2.SetLogging(false);
                m2.Disconnect();
            }
        }
    }
}