// M2UiPanel.cs (Final Corrected Version with Fixed Logic)
using UnityEngine;
using TMPro;
using UnityEngine.UI;
using System;
using System.Collections;

namespace CORC.Demo
{
    public class M2UiPanel : MonoBehaviour
    {
        [Header("Refs")]
        [Header("Participant ID")]
         public TMP_InputField participantIdIF;   // 输入受试者ID（数字）
        public Button applySIDBtn;               // 发送到下位机（S_SID）
        public CORC.CORCM2 m2;
        [Header("Auto Session (10 trials per group)")]
        public bool enableAutoSession = false;    // 勾选后启用自动化
        public int trialsPerGroup = 10;           // 每组 trial 数
        public int numGroups = 10;                // 组数（共计 trialsPerGroup * numGroups 次）
        public float preTrialCountdown = 3f;      // 每次 trial 前 UI 倒计时秒数
        public float restBetweenTrials = 5f;      // 每个 trial 结束后的停顿秒数
        public float settleAfterRsta = 2f;        // 发送 RSTA 后等待回到 A 的时间（秒）
        public Button autoStartBtn;               // 自动化开始按钮（可选）
        public Button autoStopBtn;                // 自动化停止按钮（可选）
        private IM2Proxy proxy;
        // 运行模式与累计分数
        [Header("Audio (Beep After Delay)")]
        public AudioSource audioSource;     // Assign in Inspector (e.g., on the same GameObject)
        public AudioClip beepClip;          // Optional: specific clip. If null, will call audioSource.Play()
        [Tooltip("Delay seconds for the beep module.")]
        public float beepDelaySeconds = 1f; // default 1s

        [Header("Beep Controls")]
        public Button beepAfterDelayBtn;    // optional UI button to trigger the 1s beep
        [Header("Session Rest")]
        [Tooltip("Seconds to rest after each session ends.")]
        public int restSeconds = 60;
        private bool isResting = false;

        [Header("Visual Effects")]
        public ParticleSystem windEffectLeft;
        public ParticleSystem windEffectUp;
        [Header("Per-Trial Metrics (TMP)")]
        public TMP_Text trialEffortTxt;   // 本次试次的 effort
        public TMP_Text trialDistTxt;     // 本次试次的 distance

        public TMP_Text trialDurTxt;      // 本次试次到达用时（仅 reached==1 时显示）
        [Header("Texts (TMP)")]
        public TMP_Text timeTxt;
        public TMP_Text posTxt;
        public TMP_Text velTxt;
        public TMP_Text frcTxt;
        public TMP_Text statusTxt;
        public TMP_Text countdownText;
        [Header("Buttons")]
        public Button startExperimentBtn;
        public Button startTrialBtn;
        public Button finishBtn;
        public Button resetBtn; // For the RSTA command

        [Header("Score Display")]
        public TMP_Text scoreTxt; // For dedicated score/progress display

        [Header("Scoring Controls")]
        public TMP_Dropdown scoreModeDropdown;
        public TMP_InputField maxTrialsIF;
        public TMP_InputField targetSuccIF;
        public Button applyScoreBtn;

        [Header("Probability Control")]
        public Slider probLeftSlider;
        public TMP_Text probLeftLabel;
        public Button applyProbBtn;

        private static bool TryParse(string s, out double v, double fallback = 0) { if (double.TryParse(s, out v)) return true; v = fallback; return false; }
        private static double ParseFieldD(string msg, string key, double def = 0.0) { int i = msg.IndexOf(key + "="); if (i < 0) return def; i += key.Length + 1; int j = msg.IndexOf(' ', i); string sub = (j >= 0) ? msg.Substring(i, j - i) : msg.Substring(i); if (double.TryParse(sub, out var v)) return v; return def; }
        private static string ParseFieldS(string msg, string key, string def = "") { int i = msg.IndexOf(key + "="); if (i < 0) return def; i += key.Length + 1; int j = msg.IndexOf(' ', i); string sub = (j >= 0) ? msg.Substring(i, j - i) : msg.Substring(i); return sub; }
        private bool isCountingDown = false;
        private bool autoRunning = false;         // 是否正在自动化
        private bool stopAutoRequested = false;   // 请求停止标记
        private bool lastTrialEnded = false;      // 上一次 trial 是否刚刚结束（接收到 TREN）
        public void OnStartExperimentClick()
        {
            if (proxy != null && proxy.IsReady)
            {
                proxy.SendCmd("BGIN");
                Debug.Log("[UI] Sent: BGIN");
                if (statusTxt) statusTxt.text = "Experiment Starting...";
                if (scoreTxt) scoreTxt.text = "Waiting for first trial...";
            }
        }
        /*public void OnStartTrialClick()
        {
            if (proxy != null && proxy.IsReady)
            {
                proxy.SendCmd("STRT");
                Debug.Log("[UI] Sent: STRT");
            }
        }*/
        public void OnStartTrialClick()
        {
            if (isResting||isCountingDown || proxy == null || !proxy.IsReady) return;
            StartCoroutine(CountdownAndStartTrial(3));
        }

        // 新增协程函数
        private IEnumerator SessionRestCountdown(int seconds)
        {
            isResting = true;
            if (countdownText) countdownText.gameObject.SetActive(true);

            int remaining = Mathf.Max(0, seconds);
            while (remaining > 0)
            {
                if (countdownText)
                {
                    int mm = remaining / 60;
                    int ss = remaining % 60;
                    countdownText.text = $"Rest: {mm:00}:{ss:00}";
                }
                yield return new WaitForSeconds(1f);
                remaining--;
            }

            if (countdownText)
            {
                countdownText.text = string.Empty;
                countdownText.gameObject.SetActive(false);
            }
            isResting = false; // 仅防止重复开启
        }
        private IEnumerator BeepAfterDelay(float seconds)
        {
            float wait = Mathf.Max(0f, seconds);
            yield return new WaitForSeconds(wait);

            if (audioSource)
            {
                if (beepClip)
                    audioSource.PlayOneShot(beepClip);
                else
                    audioSource.Play(); // 使用 AudioSource 上的默认 clip
            }
            else
            {
                Debug.LogWarning("[UI] BeepAfterDelay: audioSource not assigned.");
            }
        }
        private IEnumerator CountdownAndStartTrial(int seconds)
        {
            isCountingDown = true;
            if (countdownText)
            {
                countdownText.gameObject.SetActive(true);
            }

            for (int s = seconds; s > 0; s--)
            {
                if (countdownText) countdownText.text = s.ToString();
                Debug.Log($"[UI] Trial starts in {s}...");
                yield return new WaitForSeconds(1f);
            }

            if (countdownText) countdownText.text = "GO!";
            // 发送 STRT（只发一次）
            proxy.SendCmd("STRT");
            Debug.Log("[UI] Sent: STRT (after countdown)");
            yield return new WaitForSeconds(0.2f);

            if (countdownText)
            {
                countdownText.text = string.Empty;
                countdownText.gameObject.SetActive(false);
            }
            isCountingDown = false;
        }
        private IEnumerator AutoSessionRoutine()
        {
            if (proxy == null || !proxy.IsReady) yield break;
            autoRunning = true;
            if (statusTxt) statusTxt.text = "Auto session started";

            // 可选：把最大试次数设置为 trialsPerGroup * numGroups，避免早停
            int totalTrials = Mathf.Max(1, trialsPerGroup * numGroups);
            proxy.SendCmd("S_MT", new double[] { totalTrials });

            for (int g = 1; g <= numGroups && !stopAutoRequested; g++)
            {
                if (statusTxt) statusTxt.text = $"Group {g}/{numGroups}";

                for (int t = 1; t <= trialsPerGroup && !stopAutoRequested; t++)
                {
                    // 倒计时 3-2-1（可复用已有 UI）
                    yield return StartCoroutine(CountdownAndStartTrial(Mathf.RoundToInt(preTrialCountdown)));

                    // 等待机器人发回 TREN（试次结束）
                    lastTrialEnded = false;
                    float safetyTimeout = 10f; // 防御：若 trial 没有结束信号，最多等 10s
                    float waited = 0f;
                    while (!lastTrialEnded && !stopAutoRequested && waited < safetyTimeout)
                    {
                        yield return null; // 等下一帧，Update() 会把 TREN -> lastTrialEnded = true
                        waited += Time.deltaTime;
                    }

                    // 停 5 秒
                    if (restBetweenTrials > 0)
                        yield return new WaitForSeconds(restBetweenTrials);

                    // 回到 A：发 RSTA，等待若干秒（稳定时间）
                    proxy.SendCmd("RSTA");
                    if (settleAfterRsta > 0)
                        yield return new WaitForSeconds(settleAfterRsta);
                }
            }

            // 自动化完成：发送 HALT 结束 session
            proxy.SendCmd("HALT");
            if (statusTxt) statusTxt.text = "Auto session finished";
            autoRunning = false;
            stopAutoRequested = false;
        }
        public void OnAutoStartClicked()
        {
            if (!enableAutoSession) return;
            if (autoRunning) return;
            stopAutoRequested = false;
            StartCoroutine(AutoSessionRoutine());
        }

        public void OnAutoStopClicked()
        {
            stopAutoRequested = true;
        }
        public void OnFinishClick()
        {
            if (proxy != null && proxy.IsReady)
            {
                proxy.SendCmd("HALT");
                Debug.Log("[UI] Sent: HALT");
            }
        }
        public void OnApplySIDClick()
        {
            if (proxy == null || !proxy.IsReady) { Debug.LogWarning("[UI] Proxy not ready; skip S_SID"); return; }
            if (participantIdIF == null) { Debug.LogWarning("[UI] participantIdIF is null"); return; }
            var txt = participantIdIF.text?.Trim();
            if (string.IsNullOrEmpty(txt)) { if (statusTxt) statusTxt.text = "SID empty"; return; }

            // 仅允许数字（long），其他字符直接报错
            if (!long.TryParse(txt, out var sid))
            {
                if (statusTxt) statusTxt.text = $"Invalid SID: '{txt}' (must be integer)";
                Debug.LogWarning($"[UI] Invalid SID: '{txt}'");
                return;
            }

            // 发送 S_SID 命令
            proxy.SendCmd("S_SID", new double[] { (double)sid });
            Debug.Log($"[UI] Sent S_SID -> {sid}");
            if (statusTxt) statusTxt.text = $"SID set to {sid}";
        }
        public void OnResetClick()
        {
            if (proxy != null && proxy.IsReady)
            {
                proxy.SendCmd("RSTA");
                Debug.Log("[UI] Sent: RSTA");
                if (statusTxt) statusTxt.text = "Resetting to A...";
            }
        }
        public void OnApplyScore()
        {
            if (proxy == null || !proxy.IsReady) return;
            int modeIdx = scoreModeDropdown ? scoreModeDropdown.value : 0;
            double modeFlag = modeIdx + 1.0;
            proxy.SendCmd("S_MD", new double[] { modeFlag });
            Debug.Log($"[UI] Sent 'S_MD' with value: {modeFlag}");

            if (targetSuccIF && !string.IsNullOrEmpty(targetSuccIF.text))
            {
                if (TryParse(targetSuccIF.text, out var targ, 10))
                {
                    double val = Math.Max(1, (int)Math.Round(targ));
                    proxy.SendCmd("S_TS", new double[] { val });
                    Debug.Log($"[UI] Sent 'S_TS' with value: {val}");
                }
            }

            if (maxTrialsIF && !string.IsNullOrEmpty(maxTrialsIF.text))
            {
                if (TryParse(maxTrialsIF.text, out var mt, 20))
                {
                    double val = Math.Max(1, (int)Math.Round(mt));
                    proxy.SendCmd("S_MT", new double[] { val });
                    Debug.Log($"[UI] Sent 'S_MT' with value: {val}");
                }
            }
            if (statusTxt) statusTxt.text = "Parameters sent. Ready to start experiment.";
        }

        public void OnApplyProb()
        {
            if (proxy == null || !proxy.IsReady) return;
            double p = probLeftSlider ? probLeftSlider.value : 0.5;
            proxy.SendCmd("S_PB", new double[] { Mathf.Clamp01((float)p) });
            Debug.Log($"[UI] Sent 'S_PB' with value: {p:F3}");
            if (probLeftLabel) probLeftLabel.text = $"pLeft: {p:F2}";
        }
        public void OnBeepAfterDelayClick()
        {
            StartCoroutine(BeepAfterDelay(beepDelaySeconds));
        }
        void Awake()
        {
            if (m2 == null) { Debug.LogError("[M2UiPanel] Missing CORCM2 reference!"); return; }
            proxy = new M2Proxy(m2);
            if (beepAfterDelayBtn) beepAfterDelayBtn.onClick.AddListener(OnBeepAfterDelayClick);
            if (startExperimentBtn) startExperimentBtn.onClick.AddListener(OnStartExperimentClick);
            if (startTrialBtn) startTrialBtn.onClick.AddListener(OnStartTrialClick);
            if (finishBtn) finishBtn.onClick.AddListener(OnFinishClick);
            if (resetBtn) resetBtn.onClick.AddListener(OnResetClick);
            if (applyScoreBtn) applyScoreBtn.onClick.AddListener(OnApplyScore);
            if (applyProbBtn) applyProbBtn.onClick.AddListener(OnApplyProb);
            if (countdownText) { countdownText.text = string.Empty; countdownText.gameObject.SetActive(false); }
            if (probLeftLabel && probLeftSlider) probLeftLabel.text = $"pLeft: {probLeftSlider.value:F2}";
            if (trialEffortTxt) trialEffortTxt.text = string.Empty;
            if (trialDistTxt) trialDistTxt.text = string.Empty;
            if (autoStartBtn) autoStartBtn.onClick.AddListener(OnAutoStartClicked);
            if (autoStopBtn) autoStopBtn.onClick.AddListener(OnAutoStopClicked);
            if (trialDurTxt) trialDurTxt.text = string.Empty;
            if (applySIDBtn) applySIDBtn.onClick.AddListener(OnApplySIDClick);
        }

        void Update()
        {
            if (proxy == null || !proxy.IsReady)
            {
                if (statusTxt) statusTxt.text = "Connecting to Robot...";
                return;
            }

            var t = proxy.Time;
            var X = proxy.X;
            var dX = proxy.dX;
            var F = proxy.F;

            if (timeTxt) timeTxt.text = $"Time:{t:F3} s";
            if (posTxt) posTxt.text = $"Position: [{X[0]:F3}, {X[1]:F3}]";
            if (velTxt) velTxt.text = $"Velocity: [{dX[0]:F3}, {dX[1]:F3}]";
            if (frcTxt) frcTxt.text = $"Force: [{F[0]:F3}, {F[1]:F3}]";

            if (probLeftLabel && probLeftSlider)
                probLeftLabel.text = $"pLeft: {probLeftSlider.value:F2}";
            var cmds = proxy.DrainCmds();
            foreach (var c in cmds)
            {
                string cmd = (c.cmd ?? string.Empty).TrimEnd('\0');
                var p = c.parameters ?? Array.Empty<double>();
                Debug.Log($"[UI Received] {cmd} ({p.Length} params)");

                if (cmd == "ECHO")
                {
                    // 回声核对
                    Debug.Log("[UI] ECHO from M2");
                }
                else if (cmd == "TRBG")
                {
                    // params: t, dirCode, pLeft, mode, cur, max
                    int mode = (p.Length > 3) ? (int)p[3] : 0;
                    if (statusTxt) { statusTxt.color = Color.white; statusTxt.text = "Trial in progress..."; }
                    // 可选：p[1] 为方向码 -1/0/1，可驱动风特效
                }
                else if (cmd == "TREN")
                {
                    // params: t, dur, reached, dist, effort, trialScore, mode, cur, max, v1_suc, v1_tar, v2_sco, dirCode
                    int reached = (p.Length > 2) ? (int)p[2] : 0;
                    double trialScore = (p.Length > 5) ? p[5] : 0.0;
                    int mode = (p.Length > 6) ? (int)p[6] : 0;

                    if (statusTxt)
                    {
                        statusTxt.color = reached == 1 ? Color.green : Color.yellow;
                        statusTxt.text = $"Trial End! Score: {trialScore:F1}";
                    }

                    if (scoreTxt && p.Length >= 9)
                    {
                        int cur = (int)p[7];
                        int max = (int)p[8];
                        if (mode == 1) // V1
                        {
                            int v1_suc = (p.Length > 9) ? (int)p[9] : 0;
                            scoreTxt.text = $"V1: Successes {v1_suc} | Trials {cur}";
                        }
                        else if (mode == 2) // V2
                        {
                            double v2_sco = (p.Length > 12) ? p[12] : 0.0;
                            scoreTxt.text = $"V2: {cur}/{max} | Score {trialScore:F1} | Total {v2_sco:F1}";
                            // Indices: t(0), dur(1), reached(2), dist(3), effort(4), trialScore(5)


                        }
                        double effort = (p.Length > 4) ? p[4] : 0.0;
                        double dist = (p.Length > 3) ? p[3] : 0.0;
                        double dur = (p.Length > 1) ? p[1] : 0.0;

                        if (trialEffortTxt) trialEffortTxt.text = $"Effort: {effort:F3}";
                        if (trialDistTxt) trialDistTxt.text = $"Distance: {dist:F3}";


                        // Only show duration-to-reach if reached==1 (没超时才显示到达用时)
                        if (trialDurTxt)
                        {
                            if (reached == 1)
                                trialDurTxt.text = $"Time-to-reach: {dur:F3} s";
                            else
                                trialDurTxt.text = "Time-to-reach: — (timeout)";
                        }        // --- Per-trial metrics display (Effort, Distance, Score, Duration-to-reach) ---

                    }
                                    
                    lastTrialEnded = true;
                }
                else if (cmd == "SESS")
                {
                    // params: mode, V1_success, V1_trials, V2_score, V2_trials
                    int mode = (p.Length > 0) ? (int)p[0] : 0;
                    if (statusTxt) { statusTxt.color = Color.cyan; statusTxt.text = "SESSION COMPLETE!"; }
                    if (scoreTxt)
                    {
                        if (mode == 1)
                            scoreTxt.text = $"Final V1: Successes {(p.Length > 1 ? (int)p[1] : 0)} | Trials {(p.Length > 2 ? (int)p[2] : 0)}";
                        else if (mode == 2)
                            scoreTxt.text = $"Final V2: Total {(p.Length > 3 ? p[3] : 0.0):F1} | Trials {(p.Length > 4 ? (int)p[4] : 0)}";
                    }
                    // 在 SESS 分支末尾添加：
                    if (!isResting && restSeconds > 0)
                    {
                        StartCoroutine(SessionRestCountdown(restSeconds));
                    }
                    autoRunning = false; // 自动化结束
                    stopAutoRequested = true; // 确保停止标记
                }
            }

    }
    }

    public static class FLNLClientExtensions
    {
        public static System.Collections.Generic.List<string> ReadMessages(this FLNLClient client)
        {
            var messages = new System.Collections.Generic.List<string>();
            while (client.IsReceivedCmd())
            {
                messages.Add(client.GetReceivedCmd().cmd);
            }
            return messages;
        }
    }
}


