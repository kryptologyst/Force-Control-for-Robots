# DISCLAIMER.md

## ⚠️ IMPORTANT SAFETY NOTICE

**THIS SOFTWARE IS FOR RESEARCH AND EDUCATIONAL PURPOSES ONLY**

### DO NOT USE ON REAL HARDWARE WITHOUT EXPERT REVIEW

This force control system is designed for simulation, research, and educational purposes. It has NOT been tested on real robotic hardware and may cause:

- **Physical damage** to robots, objects, or environment
- **Personal injury** to operators or bystanders
- **Unsafe robot behavior** including unexpected movements
- **System failures** that could lead to dangerous situations

### SAFETY REQUIREMENTS FOR REAL HARDWARE USE

If you intend to use this software on real robotic hardware, you MUST:

1. **Expert Review**: Have the system reviewed by robotics safety experts
2. **Safety Testing**: Conduct comprehensive safety testing in controlled environments
3. **Risk Assessment**: Perform thorough risk assessment and mitigation
4. **Safety Systems**: Implement proper safety systems including:
   - Emergency stop mechanisms
   - Force and velocity limits
   - Collision detection and avoidance
   - Watchdog timers and monitoring
   - Physical barriers and safety zones
5. **Training**: Ensure all operators are properly trained
6. **Certification**: Obtain necessary safety certifications
7. **Insurance**: Maintain appropriate liability insurance

### LIMITATIONS AND ASSUMPTIONS

This software makes several assumptions that may not hold in real-world scenarios:

- **Perfect sensors**: Assumes ideal force/torque sensors with no noise or delays
- **Perfect actuators**: Assumes ideal actuators with instant response
- **Rigid bodies**: Assumes rigid robot links (no flexibility or compliance)
- **No friction**: Simplified friction models
- **No disturbances**: No external disturbances or uncertainties
- **Perfect models**: Assumes perfect robot and environment models

### KNOWN LIMITATIONS

- **No collision detection** in the basic implementation
- **Limited error handling** for sensor failures
- **No real-time guarantees** for control loop timing
- **Simplified physics** that may not match real robot behavior
- **No safety interlocks** or emergency stop integration
- **Limited validation** of control commands before execution

### RECOMMENDED SAFETY PRACTICES

Even in simulation, follow these safety practices:

1. **Start with low gains** and gradually increase
2. **Monitor all signals** for unexpected behavior
3. **Set conservative limits** for forces and velocities
4. **Test extensively** before considering real hardware
5. **Document all changes** and test results
6. **Use version control** for all code modifications

### LIABILITY DISCLAIMER

The authors and contributors of this software:

- **DISCLAIM ALL LIABILITY** for any damages, injuries, or losses
- **MAKE NO WARRANTIES** about the software's safety or suitability
- **ARE NOT RESPONSIBLE** for any misuse or misapplication
- **RECOMMEND PROFESSIONAL REVIEW** before any real-world use

### CONTACT

For questions about safety or real-world applications, please contact robotics safety experts or professional robotics consultants.

---

**Remember: Safety first! When in doubt, don't use on real hardware.**
