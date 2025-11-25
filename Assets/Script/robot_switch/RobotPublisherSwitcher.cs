using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

[AddComponentMenu("Robots/Robot Publisher Switcher (Enable/Disable, Safe)")]
public class RobotPublisherSwitcher : MonoBehaviour
{
    [Serializable]
    public class RobotGroup
    {
        public string name = "robot";
        [Tooltip("拖入要切換的『Publisher 元件本身』，若誤拖 RosConnector 會自動忽略它並尋找同物件兄弟 Publisher。")]
        public Behaviour[] publishers;
    }

    [Header("Switch order: viper → stretch3 → koch")]
    public RobotGroup viper = new RobotGroup { name = "viper" };
    public RobotGroup stretch3 = new RobotGroup { name = "stretch3" };
    public RobotGroup koch = new RobotGroup { name = "koch" };

    [Header("Input")]
    public KeyCode switchKey = KeyCode.Space;

    [Header("Advanced")]
    [Tooltip("啟用 Publisher 前，若其物件/父物件是 Inactive，會一路 SetActive(true)。")]
    public bool activateHierarchyOnEnable = true;

    [Tooltip("為避免其他腳本在同一幀把它關回去，下一幀再強制同步一次狀態。")]
    public bool reapplyOnNextFrame = true;

    // === Stretch3 專用 ===
    [Header("Stretch3 專用")]
    [Tooltip("切換到 stretch3 時，這些元件會被 Disable；離開 stretch3 時會依下方選項還原。")]
    public Behaviour[] disableWhileStretch3Active;
    [Tooltip("離開 stretch3 後是否自動重新啟用上面列出的元件。")]
    public bool reenableWhenLeavingStretch3 = true;

    // === 新增 Stretch3 GameObject 控制（多物件） ===
    [Tooltip("切換到 stretch3 之外時會自動 Disable；切回 stretch3 時自動 Enable。可放多個 GameObject。")]
    public GameObject[] stretch3OnlyObjects;

    // 0=viper,1=stretch3,2=koch
    [SerializeField] private int currentIndex = 0;

    private RobotGroup[] groups;
    private List<Behaviour>[] resolved; // 每組實際要被切換的 Publisher 清單

    private void Awake()
    {
        groups = new[] { viper, stretch3, koch };
        ResolveAll();
        ApplyActive(0, log: true); // 預設 viper
    }

    private void Update()
    {
        if (Input.GetKeyDown(switchKey))
        {
            int next = (currentIndex + 1) % groups.Length;
            ApplyActive(next, log: true);
        }
    }

    // --- 解析 Inspector 內容 → 真正要切的 Publisher（自動略過/展開 Connector） ---
    private void ResolveAll()
    {
        resolved = new List<Behaviour>[3];
        for (int gi = 0; gi < 3; gi++)
        {
            var list = new List<Behaviour>();
            var src = groups[gi].publishers ?? Array.Empty<Behaviour>();

            foreach (var b in src)
            {
                if (!b) continue;

                if (IsConnector(b))
                {
                    var sibs = b.GetComponents<Behaviour>().Where(x => x && !IsConnector(x) && x != this);
                    list.AddRange(sibs);
                }
                else
                {
                    list.Add(b);
                }
            }

            resolved[gi] = list.Where(x => x).Distinct().ToList();

            if (resolved[gi].Count == 0)
                Debug.LogWarning($"[Switcher] '{groups[gi].name}' 沒解析到 Publisher。");
            else
                Debug.Log($"[Switcher] '{groups[gi].name}' 解析：{string.Join(" | ", resolved[gi].Select(PathOf))}");
        }
    }

    private static bool IsConnector(Behaviour b)
    {
        var n = b.GetType().Name;
        return n.IndexOf("RosConnector", StringComparison.OrdinalIgnoreCase) >= 0
            || n.IndexOf("ROSConnection", StringComparison.OrdinalIgnoreCase) >= 0
            || n.IndexOf("RosConnection", StringComparison.OrdinalIgnoreCase) >= 0;
    }

    // --- 核心切換 ---
    private void ApplyActive(int targetIndex, bool log)
    {
        if (resolved == null || resolved.Length != 3) ResolveAll();

        for (int gi = 0; gi < resolved.Length; gi++)
        {
            bool enable = (gi == targetIndex);

            foreach (var comp in resolved[gi])
            {
                if (!comp) continue;

                if (enable && activateHierarchyOnEnable && !comp.gameObject.activeInHierarchy)
                    ActivateHierarchy(comp.gameObject);

                comp.enabled = enable;

                if (reapplyOnNextFrame)
                    StartCoroutine(ReapplyNextFrame(comp, enable));
            }
        }

        // Stretch3 專用元件開關
        bool stretch3Active = (targetIndex == 1);

        if (disableWhileStretch3Active != null && disableWhileStretch3Active.Length > 0)
        {
            foreach (var beh in disableWhileStretch3Active)
            {
                if (!beh) continue;
                bool target = stretch3Active ? false : (reenableWhenLeavingStretch3 ? true : beh.enabled);
                beh.enabled = target;
                if (reapplyOnNextFrame)
                    StartCoroutine(ReapplyNextFrame(beh, target));
            }
        }

        // === 新增：Stretch3 GameObject 多物件控制 ===
        if (stretch3OnlyObjects != null && stretch3OnlyObjects.Length > 0)
        {
            foreach (var go in stretch3OnlyObjects)
            {
                if (!go) continue;
                go.SetActive(stretch3Active);
                Debug.Log($"[Switcher][Stretch3] {(stretch3Active ? "Enable" : "Disable")} GameObject '{go.name}'");
            }
        }

        int prev = currentIndex;
        currentIndex = targetIndex;
        if (log) Debug.Log($"[Switcher] {groups[prev].name} → {groups[currentIndex].name}");
    }

    private static void ActivateHierarchy(GameObject go)
    {
        var stack = new List<Transform>();
        var t = go.transform;
        while (t != null)
        {
            stack.Add(t);
            t = t.parent;
        }
        for (int i = stack.Count - 1; i >= 0; i--)
        {
            var g = stack[i].gameObject;
            if (!g.activeSelf) g.SetActive(true);
        }
    }

    private System.Collections.IEnumerator ReapplyNextFrame(Behaviour comp, bool enable)
    {
        yield return null;
        if (comp) comp.enabled = enable;
    }

    private static string PathOf(Behaviour b)
    {
        var t = b.transform;
        var stack = new List<string>();
        while (t) { stack.Add(t.name); t = t.parent; }
        stack.Reverse();
        return string.Join("/", stack) + $"[{b.GetType().Name}]";
    }

#if UNITY_EDITOR
    private void OnValidate() { groups = new[] { viper, stretch3, koch }; }
#endif
}
