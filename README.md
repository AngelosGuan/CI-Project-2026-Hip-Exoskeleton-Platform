# CI-Project-2026-Hip-Exoskeleton-Platform
 This project focuses on developing a hip‑actuated exoskeleton prototype to support rapid experimentation in lower‑limb control research.




# KT for current control for ak80-9 kv100 
 0.095 Nm/A
# Use <24V to power the motor for testing


# for demo code with IMUs

need 2 imus,  

1: ( placed on Right thigh, black wire goes to ground pin20, blue goes to pin28)

2: ( placed on Left thigh, black wire goes to ground pin6, blue goes to pin10)

need to install wiringPi library

execute 

<code>./main</code>

compile with 

<code>gcc main.c -o main OscAddress.c OscBundle.c OscCommon.c OscError.c OscMessage.c OscPacket.c OscSlip.c NgimuReceive.c filter.c  -lm -lwiringPi</code>


<img width="2064" height="1185" alt="GPIO" src="https://github.com/user-attachments/assets/8e95d916-5695-429a-9ded-6ed463742805" />
