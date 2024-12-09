
# Example Scenes: Connecting to the Real Arm  

These scenes demonstrate how to:  
- Connect to the real Koch arm  
- Retrieve the arm’s status  
- Control the arm  

---

## Prerequisites  

1. **ROS# Requirement:** Ensure your ROS# version is **2.0.0 or later** for ROS2 communication.  
2. **Real Robot Setup:** Follow the instructions in **[koch_ros2_wrapper](https://github.com/hrc-pme/koch_ros2_wrapper)** for real robot configuration.  

---

## Important Components  

**Key GameObject:** `XXX_connector_OOO` (an empty GameObject with ROS connector scripts).  

### Important Parameters  

- **Ros Connector:**  
  - **Ros Bridge Server URL:** Format `ws://{IP}:{port}` (e.g., `ws://192.168.0.244:9090`)  
  - **ROS Version:** Specify the ROS version (`ROS1` or `ROS2`).  

- **Publisher/Subscriber:**  
  - **Topic:** Specify the correct topic names for publishing or subscribing.  

⚠️ **Important:** Double-check IPs and topic names. Typos, extra spaces, or incorrect indentation may cause connection issues.  

---

## Example Scenes  

### **Example 1: Control from Leader Example**  

- **Scenario:** The Unity arm acts as a simulator controlled by the real leader arm.  

**Setup:**  
- Real Arm: Follow the **[single_leader_control guide](https://github.com/hrc-pme/koch_ros2_wrapper#setup-1-single-leader)**.  
- Unity: Verify IP and topic name settings.  

---

### **Example 2: Status from Follower Example**  

- **Scenario:** Unity retrieves joint state data from the real follower arm, providing a virtual representation.  

**Setup:**  
- Real Arm: Follow the **[leader_follower_control guide](https://github.com/hrc-pme/koch_ros2_wrapper#setup3-single-leader-follower)**.  
- Unity: Ensure IP and topic names are correct.  

---

### **Example 3: Keyboard Control Example**  

- **Scenario:** Control the arm using the keyboard. Use keys `A-Z, S-X, D-C, F-V, G-B, H-N` for joints 1-6. Press `Space` to send joint state data to the real arm.  

**Modes:**  
- **Goal Point Control:** Press once.  
- **Dragging Control:** Hold the keys.  

**Setup:**  
- Real Arm: Follow the **[single_follower_control guide](https://github.com/hrc-pme/koch_ros2_wrapper#setup2-single-follower)**.  
- Unity: Double-check IP and topic names.  

---

### **Example 4: Leader-Follower Example**  

- **Scenario:** Unity forwards joint states from the leader arm to the follower arm.  

**Control Flow Comparison:**  
- **Single Leader-Follower:** `Leader → Follower` (same machine).  
- **Leader-Follower Example:** `Leader → Unity → Follower` (can use different machines).  

**Setup:**  
- Real Arm: Follow the **[leader_follower_control guide](https://github.com/hrc-pme/koch_ros2_wrapper#setup3-single-leader-follower)**.  
- Unity: Verify IP and topic names.  

---
