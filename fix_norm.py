import sys

with open('APP/Chassis.c', 'r') as f:
    lines = f.readlines()

# Find NORMAL_MODE case
case_idx = None
for i, line in enumerate(lines):
    if 'case NORMAL_MODE:' in line:
        case_idx = i
        break

if case_idx is None:
    print('ERROR: NORMAL_MODE case not found')
    sys.exit(1)

# Find the break statement after this case
break_idx = None
for i in range(case_idx+1, min(case_idx+20, len(lines))):
    if 'break;' in lines[i]:
        break_idx = i
        break

if break_idx is None:
    print('ERROR: break not found after NORMAL_MODE')
    sys.exit(1)

print(f'NORMAL_MODE case at line {case_idx+1}, break at {break_idx+1}')

# Build replacement
new = []
new.append('\t\t\tcase NORMAL_MODE:\n')
new.append('\t\t\t\t// ====== \xd4\xad\xb4\xfa\xc2\xeb\xa3\xa8\xd7\xa2\xca\xcd\xb5\xf4\xa3\xac\xb5\xf7\xcd\xea\xcb\xd9\xb6\xc8\xbb\xb7\xbb\xd6\xb8\xb4\xa3\xa9 ======\n')
new.append('\t\t\t\t// static int init_flag=1;\n')
new.append('\t\t\t\t// if (init_flag) {\n')
new.append('\t\t\t\t// \tChassis_Set_Line(1);\n')
new.append('\t\t\t\t// \tinit_flag=0;\n')
new.append('\t\t\t\t// }\n')
new.append('\t\t\t\t// ====== \xcb\xd9\xb6\xc8\xbb\xb7\xb5\xf7\xca\xd4\xb4\xfa\xc2\xeb ======\n')
new.append('\t\t\t\t{\n')
new.append('\t\t\t\t\tstatic int init_flag = 1;\n')
new.append('\t\t\t\t\tif (init_flag) {\n')
new.append('\t\t\t\t\t\tmotor_l->loop_mode = SPEED_MODE;\n')
new.append('\t\t\t\t\t\tmotor_r->loop_mode = SPEED_MODE;\n')
new.append('\t\t\t\t\t\tmotor_l->State = ENABLE;\n')
new.append('\t\t\t\t\t\tmotor_r->State = ENABLE;\n')
new.append('\t\t\t\t\t\tPID_clear(&motor_l->speed_pid);\n')
new.append('\t\t\t\t\t\tPID_clear(&motor_r->speed_pid);\n')
new.append('\t\t\t\t\t\tLine_flag = 0;\n')
new.append('\t\t\t\t\t\tinit_flag = 0;\n')
new.append('\t\t\t\t\t}\n')
new.append('\n')
new.append('\t\t\t\t\t/* Ozone: \xd6\xb1\xbd\xd3\xb8\xc4 debug_speed_ref \xb5\xc4\xd6\xb5\xc0\xb4\xb2\xe2\xcb\xd9\xb6\xc8\xbd\xd7\xd4\xbe\xcf\xec\xd3\xa6 */\n')
new.append('\t\t\t\t\tvolatile float debug_speed_ref = 0.0f;\n')
new.append('\n')
new.append('\t\t\t\t\tDC_Motor_SetRef(motor_l, debug_speed_ref);\n')
new.append('\t\t\t\t\tDC_Motor_SetRef(motor_r, debug_speed_ref);\n')
new.append('\t\t\t\t\tDCMotor_SetTraceCompensation(motor_l, 0.0f);\n')
new.append('\t\t\t\t\tDCMotor_SetTraceCompensation(motor_r, 0.0f);\n')
new.append('\t\t\t\t}\n')
new.append('\t\t\t\tbreak;\n')

# Replace
result = lines[:case_idx] + new + lines[break_idx+1:]

with open('APP/Chassis.c', 'w') as f:
    f.writelines(result)

print('Done')
