# ME499 Winter 2025: Biomimetic Cable-Driven Dexterous Hand  
**Author:** Zhengyang Kris Weng 

## Objective  
To design and prototype a biomimetic robotic hand with 10+ degrees of freedom (target: 15 DOF), capable of open-loop control for grasping and manipulating everyday objects. The system will prioritize human-like joint allocation, lightweight construction, and adaptability to diverse object shapes, with potential extensions into closed-loop control using vision-based feedback.

![image](https://github.com/user-attachments/assets/f32be675-9c71-47fe-bf30-9836b11647a3)




## Background  
The human hand, with its 27 bones, intricate network of tendons and muscles, and over 20 degrees of freedom, is a marvel of dexterity and adaptability. It seamlessly transitions between delicate tasks, like threading a needle, and powerful actions, such as gripping heavy objects. Inspired by this remarkable complexity, I aim to design a robotic hand that not only replicates its functionality but also captures its versatility and precision.

![hand anatomy](https://github.com/user-attachments/assets/ce447bf6-88ae-414c-a4d4-d9c5f0e136c8)

Cable-driven dexterous robotic hands replicate the intricate movements of the human hand using lightweight cable-based actuation systems. These systems are designed for applications requiring precision and adaptability, such as rehabilitation, surgical assistance, and research. By leveraging anthropomorphic features, underactuation, and compliance, they can perform complex grasping tasks with fewer actuators than degrees of freedom, reducing weight and mechanical complexity while maintaining high dexterity. This adaptability makes them particularly effective for interacting with diverse objects in clinical and surgical environments.  

However, cable-driven mechanisms pose significant challenges in both mechanical design and control. Achieving an optimal solution often requires extensive tuning and iteration, which can be difficult within the limited timeframe of this project. Additionally, constructing an accurate kinematic model for such a complex system is another key challenge that demands careful consideration.  

## Project Scope  
The goal is to develop a **10+ Degree of Freedom (DOF)** biomimetic robotic hand (target: 15 DOF) with **3 DOF per finger digit and thumb**. The project will exclude wrist functionality and focus on the hand itself.  

The joints will be cable-driven to minimize inertia on the distal end, and the hand should achieve at least open-loop control of the finger digits. A high-level controller will interface with the hand to manage its operations, while motor control signals will be transmitted through a communication bus.

### Core Requirements:  
- Open-loop control of finger digits.  
   - Capability to grasp everyday objects (e.g., water bottles).  
   - Ability to manipulate objects within the hand.  
   - Capability to perform various hand gestures.  
- A design resembling human hand anatomy in terms of joint allocation, size, and weight.
   - Maximum system weight of 3kg (max payload of franka FER)
- An interface that can mount to a [Franka Emika Panda Robot Arm](https://franka.de/)  
![image](https://github.com/user-attachments/assets/ef81da8e-5b9c-4d07-aa47-845e73d9f91a)


### Stretch Goals:  
1. **Closed-Loop Control with Sensing:**  
   - Implement vision-based pose estimation and control.  
   - Use vision-based hand tracking and shadowing.  

   Instead of in-situ joint-level sensing through encoders or potentiometers, the system will rely on a combination of vision-based skeleton detection and joint sensing, complemented by open-loop commands to the joints. This approach mimics human proprioceptive sensing with vision compensation, aligning with the biomimetic theme. A Kalman filter can also be introduced to enhance control accuracy. This goal may extend into the final project of my MSR program.

![image](https://github.com/user-attachments/assets/fdcb88bb-12a9-4eb4-ac69-823a961f928c)


3. **Touch Sensing (Stretch Goal+):**  
   - Incorporate touch sensors to improve interaction capabilities.  
   - This goal may extend into the final project of my MSR program.  

## Deliverables  
1. **Functional Prototype**  
   - A working robotic hand capable of open-loop control, grasping, and manipulation.  

2. **Documentation**  
   - Detailed design and implementation report, including challenges, results, and future recommendations.  

3. **CAD Source Files**  
   - Complete CAD models of the robotic hand, including all mechanical components.  

4. **Code and scripts**
   - Any software code or scripts used to control the robotic hand.

5. **Sample Task Demo**
   - Video or live demo of the robot hand executing tasks.

## Evaluation Metrics  
- **Functionality:** Ability to achieve open-loop control of individual finger digits.  
- **Grasping and Manipulation:** Successful grasping and manipulation of everyday objects.  
- **Design Accuracy:** Adherence to human hand anatomy in terms of joint allocation, size, and weight.  
- **Adaptability:** Ability to perform a variety of hand gestures and interact with objects of different shapes and sizes.  

---

## Addendum: Milestones and Timeline  

### **Week 1: Initial Planning and Research**  
- Finalize system architecture and design requirements.  
- Conduct research on similar robotic hands and control systems.  
- Select components (motors, cables, controllers, etc.).  

### **Week 2: CAD Design and Simulations**  
- Create CAD models of the hand, focusing on joint and cable placement.  
- Simulate the mechanism for feasibility (e.g., joint range of motion, cable tension).  

### **Week 3–4: Hardware Prototyping**  
- Fabricate or 3D print components.  
- Assemble the hand structure, including joints and cable routing.  
- Begin initial integration of motors and communication systems.  

### **Week 5: Basic Control Implementation**  
- Implement open-loop control for individual finger digits.  
- Test movement and refine joint actuation mechanisms.  

### **Week 6: Grasping Tests**  
- Test the ability to grasp and hold objects (e.g., water bottles, small tools).  
- Adjust cable tension, control parameters, or hand design as needed.  

### **Week 7: High-Level Controller**  
- Develop and integrate a high-level control interface.  
- Ensure smooth communication between the controller and motors.  

### **Week 8–9: Stretch Goals (Closed-Loop Control and Vision Integration)**  
- Implement vision-based pose estimation and tracking.  
- Introduce closed-loop feedback for improved control accuracy.  
- Experiment with hand gestures and object manipulation.  

### **Week 10: Final Testing and Documentation**  
- Conduct final functionality tests (e.g., grasping, gestures, manipulation).  
- Prepare a report or presentation summarizing results, challenges, and future directions.  
