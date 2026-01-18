using UnityEngine;

public class InspectorNote : MonoBehaviour
{
    [Header("Notes")]
    [Tooltip("這裡只用來寫備註，不會影響任何邏輯")]
    [TextArea(3, 10)]
    public string note;
}
