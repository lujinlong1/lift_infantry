"""Audit actual ARM linker output; no third-party packages or target access."""
from pathlib import Path
import re
import sys
root = Path(sys.argv[1]).resolve() if len(sys.argv)>1 else Path(__file__).resolve().parents[2]
map_text = (root/'MDK-ARM/lift_infantry/lift_infantry.map').read_text(encoding='utf-8')
html = (root/'MDK-ARM/lift_infantry/lift_infantry.htm').read_text(encoding='utf-8')
symbols = {}
for name,address,size,obj in re.findall(r'^\s*([\w.]+)\s+(0x[0-9a-fA-F]+)\s+Data\s+(\d+)\s+(\S+)\(',map_text,re.M):
    symbols[name] = {'address':int(address,16),'bytes':int(size),'object':obj}
regions = {}
for name,base,size,maximum in re.findall(r'Execution Region (RW_IRAM\d).*?Exec base: (0x\w+).*?Size: (0x\w+), Max: (0x\w+)',map_text):
    regions[name] = {'base':int(base,16),'used':int(size,16),'maximum':int(maximum,16)}
    assert regions[name]['used'] <= regions[name]['maximum']
for name in ['dbus_dma_buffer','telemetry_frame']:
    symbol=symbols[name]
    assert 0x20000000<=symbol['address'] and symbol['address']+symbol['bytes']<=0x20020000, (name,symbol)
print('DMA buffers:')
for name in ['dbus_dma_buffer','telemetry_frame']:
    print('  {}: 0x{:08X}, {} bytes, SRAM DMA-accessible'.format(name,symbols[name]['address'],symbols[name]['bytes']))
for name,r in regions.items():
    print('{}: used {}, free {}, maximum {}'.format(name,r['used'],r['maximum']-r['used'],r['maximum']))
functions = [
 ('can_rx_task','can_rx_task_stack'),('robot_motor_tx_task','motor_tx_task_stack'),
 ('lift_test_task','lift_test_task_stack'),('chassis_control_task','chassis_control_task_stack'),
 ('friction_task','friction_control_task_stack'),('feed_task','feed_task_stack'),
 ('shooter_task','shooter_task_stack'),('yaw_task','yaw_task_stack'),('pitch_task','pitch_task_stack'),
 ('lift_control_task','lift_task_stack'),('remote_task','remote_task_stack'),
 ('telemetry_task','telemetry_task_stack'),('link_task','task_stack'),
 ('runtime_monitor_task','runtime_monitor_task_stack'),('StartSystemTask',None),
 ('prvIdleTask','vApplicationGetIdleTaskMemory.Idle_Stack'),
 ('prvTimerTask','vApplicationGetTimerTaskMemory.Timer_Stack')]
print('Task, allocated bytes, Keil max depth, reserve, residual:')
for function,symbol in functions:
    m=re.search(r'</a>'+function+r'</STRONG>(.*?)</UL>',html,re.S)
    assert m,function
    depth=int(re.search(r'Max Depth =\s*(\d+)',m.group(1)).group(1))
    if symbol:
        allocation=symbols[symbol]['bytes']
    else:
        source=(root/'Src/freertos.c').read_text(encoding='utf-8')
        block=re.search(r'systemTask_attributes\s*=\s*\{(.*?)\};',source,re.S).group(1)
        allocation=int(re.search(r'stack_size\s*=\s*(\d+)\s*\*\s*4',block).group(1))*4
    reserve=1024 if function=='lift_control_task' else (256 if function=='prvIdleTask' else 512)
    residual=allocation-depth-reserve
    print('  {}, {}, {}, {}, {}'.format(function,allocation,depth,reserve,residual))
    assert residual>=0, function+' static stack reserve exceeded'
    if residual<256:print('  REVIEW: {} has <256B residual after reserve; runtime high-water test required'.format(function))
for name in ['g_lift_blackbox','g_lift_diag','g_lift_diag_transport','lift_diag_frame','snapshot','ucHeap','Stack_Mem','Heap_Mem']:
    s=symbols[name]
    print('{}: 0x{:08X}, {} bytes'.format(name,s['address'],s['bytes']))
print('PASS: actual SRAM/CCM/DMA placement and task static stack reserves')
print('LIMIT: static call graph excludes runtime scheduling, ISR nesting and unresolved callbacks; no WCET claim')
