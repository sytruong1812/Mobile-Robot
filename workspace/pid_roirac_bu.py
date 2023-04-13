import pid_roirac_vitri_phai
import pid_roirac_vitri_trai

vitriphai = pid_roirac_vitri_phai.vitri
vitritrai = pid_roirac_vitri_trai.vitri

E = 0; E1 = 0; E2 = 0           # Sai số
output_bu = 0; last_output_bu = 0     # PWM điều khiển động cơ
alpha = 0; beta = 0; gama = 0 

Kp = 0.03; Ki = 0.00005; Kd = 0.00005
T = 30           # Thời gian lấy mẫu (Quan trọng)
PPR = 2300       # Encoder đo được 2390 xung / vòng

def PID_roirac_vitri_phai(output_bu):
    global E, E1, E2, last_output_bu
    global alpha, beta, gama
    
    # Tính sai số E
    if(vitriphai > vitritrai): 
        E = abs(vitriphai) - abs(vitritrai)
    if(vitriphai < vitritrai):
        E = abs(vitritrai) - abs(vitriphai)

    alpha = (2 * T * Kp) + (Ki * T * T) + (2 * Kd)
    beta = (T * T * Ki) - (4 * Kd) - (2 * T * Kp)
    gama = 2 * Kd
    output_bu = (alpha*E + beta*E1 + gama*E2 + 2*T*last_output_bu) / (2*T)
    last_output_bu = output_bu
    E2 = E1
    E1 = E 

    if(output_bu > 40):
        output_bu = 40
    if(output_bu < 30):
        output_bu = 0
    if(output_bu > 0):
        output_bu = output_bu
    return output_bu
